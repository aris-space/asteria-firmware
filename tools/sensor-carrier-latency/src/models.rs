//! The three fits. A sample stamped `t` by a sensor with latency `τ` measures
//! the signal at `t − τ`; all latencies are relative to IMU_0's timestamps.

use crate::fit::{Model, Row};
use crate::log::{
    Baro, Gnss, Imu, Log, Mag, STANDARD_GRAVITY, decimate, noise_sigma, pressure_altitude_m,
};
use crate::spline::{Basis, Spline};

// Latency-shifted times must stay inside the spline.
const MARGIN_S: f64 = 0.5;

/// IMU_1 against IMU_0: both see the same board rotation.
pub struct ImuTwin {
    spline: Spline,
    reference: Vec<Imu>,
    other: Vec<Imu>,
    sigma_dps: [[f64; 3]; 2],
}

impl ImuTwin {
    const KNOT_S: f64 = 0.05;

    pub fn new(log: &Log) -> Option<Self> {
        let reference = decimate(&log.imu[0], 4);
        let other = decimate(&log.imu[1], 4);
        let (start, end) = span(reference.iter().map(|s| s.t))?;
        other.first()?;
        let sigma = |samples: &[Imu]| {
            std::array::from_fn(|axis| noise_sigma(samples.iter().map(|s| s.gyro_dps[axis]), 1e-3))
        };
        Some(Self {
            spline: Spline::new(start, end, Self::KNOT_S, MARGIN_S),
            sigma_dps: [sigma(&reference), sigma(&other)],
            reference,
            other,
        })
    }

    fn bias(&self, axis: usize) -> usize {
        3 * self.spline.count() + axis
    }
}

impl Model for ImuTwin {
    fn unknowns(&self) -> usize {
        3 * self.spline.count() + 3
    }

    fn latency_names(&self) -> Vec<String> {
        vec!["IMU_1".into()]
    }

    fn rows(&self, latencies: &[f64]) -> Vec<Row> {
        let mut rows = Vec::new();
        for sample in &self.reference {
            let basis = self.spline.basis(sample.t);
            for axis in 0..3 {
                rows.push(Row {
                    entries: on_axis(&basis, &basis.value, axis, 1.0),
                    rhs: sample.gyro_dps[axis],
                    sigma: self.sigma_dps[0][axis],
                    d_latency: None,
                });
            }
        }
        for sample in &self.other {
            let basis = self.spline.basis(sample.t - latencies[0]);
            for axis in 0..3 {
                let mut entries = on_axis(&basis, &basis.value, axis, 1.0);
                entries.push((self.bias(axis), 1.0));
                rows.push(Row {
                    entries,
                    rhs: sample.gyro_dps[axis],
                    sigma: self.sigma_dps[1][axis],
                    d_latency: Some((0, on_axis(&basis, &basis.d1, axis, -1.0))),
                });
            }
        }
        rows
    }
}

/// One magnetometer against IMU_0's gyro. In the board frame the field turns
/// opposite to the board, `ṁ = −ω × (m − h)`, with `h` a residual hard-iron
/// offset that the stored calibration did not remove.
pub struct MagRotation {
    name: String,
    spline: Spline,
    gyro: Vec<Imu>,
    samples: Vec<Mag>,
    field_sigma_nt: [f64; 3],
    rate_sigma_nt_per_s: f64,
}

impl MagRotation {
    const KNOT_S: f64 = 0.2;

    pub fn new(log: &Log, index: usize) -> Option<Self> {
        let samples = log.mag[index].clone();
        let (start, end) = span(samples.iter().map(|s| s.t))?;
        let gyro = decimate(&log.imu[0], 16)
            .into_iter()
            .filter(|s| (start..=end).contains(&s.t))
            .collect::<Vec<_>>();
        gyro.first()?;
        // Gyro noise turns into field-rate noise through |m|.
        let gyro_sigma_rad_s = (0..3)
            .map(|axis| noise_sigma(gyro.iter().map(|s| s.gyro_dps[axis].to_radians()), 1e-5))
            .fold(0.0, f64::max);
        let mut magnitudes: Vec<f64> = samples
            .iter()
            .map(|s| s.field_nt.iter().map(|v| v * v).sum::<f64>().sqrt())
            .collect();
        magnitudes.sort_by(f64::total_cmp);
        let field = magnitudes[magnitudes.len() / 2];
        Some(Self {
            name: format!("MAG_BUS_{}", index + 1),
            spline: Spline::new(start, end, Self::KNOT_S, MARGIN_S),
            field_sigma_nt: std::array::from_fn(|axis| {
                noise_sigma(samples.iter().map(|s| s.field_nt[axis]), 1.0)
            }),
            rate_sigma_nt_per_s: (gyro_sigma_rad_s * field).max(1.0),
            gyro,
            samples,
        })
    }

    fn offset(&self, axis: usize) -> usize {
        3 * self.spline.count() + axis
    }
}

impl Model for MagRotation {
    fn unknowns(&self) -> usize {
        3 * self.spline.count() + 3
    }

    fn latency_names(&self) -> Vec<String> {
        vec![self.name.clone()]
    }

    fn rows(&self, latencies: &[f64]) -> Vec<Row> {
        let mut rows = Vec::new();
        for sample in &self.gyro {
            let basis = self.spline.basis(sample.t);
            let w = sample.gyro_dps.map(f64::to_radians);
            // (ω × v)_axis = ω[a]·v[b] − ω[b]·v[a] for the axis's cyclic pair (a, b).
            for axis in 0..3 {
                let (a, b) = ((axis + 1) % 3, (axis + 2) % 3);
                let mut entries = on_axis(&basis, &basis.d1, axis, 1.0);
                entries.extend(on_axis(&basis, &basis.value, b, w[a]));
                entries.extend(on_axis(&basis, &basis.value, a, -w[b]));
                entries.push((self.offset(b), -w[a]));
                entries.push((self.offset(a), w[b]));
                rows.push(Row {
                    entries,
                    rhs: 0.0,
                    sigma: self.rate_sigma_nt_per_s,
                    d_latency: None,
                });
            }
        }
        for sample in &self.samples {
            let basis = self.spline.basis(sample.t - latencies[0]);
            for axis in 0..3 {
                rows.push(Row {
                    entries: on_axis(&basis, &basis.value, axis, 1.0),
                    rhs: sample.field_nt[axis],
                    sigma: self.field_sigma_nt[axis],
                    d_latency: Some((0, on_axis(&basis, &basis.d1, axis, -1.0))),
                });
            }
        }
        rows
    }
}

/// Barometers and GNSS against IMU_0's vertical acceleration, all on one
/// height trajectory. Each barometer has a constant offset from MSL.
pub struct Vertical {
    spline: Spline,
    accel: Vec<(f64, f64)>,
    accel_sigma_mps2: f64,
    baro: Vec<(usize, Vec<Baro>, f64)>,
    gnss: Vec<(usize, Vec<Gnss>)>,
}

impl Vertical {
    const KNOT_S: f64 = 0.1;
    const MIN_SIGMA: f64 = 0.05;

    pub fn new(log: &Log) -> Option<Self> {
        let accel = decimate(&log.imu[0], 8)
            .into_iter()
            .filter_map(|s| {
                let q = log.attitude_at(s.t)?;
                let f_down = rotate(q, s.accel_g.map(|g| g * STANDARD_GRAVITY))[2];
                Some((s.t, -(f_down + STANDARD_GRAVITY)))
            })
            .collect::<Vec<_>>();
        let (start, end) = span(accel.iter().map(|&(t, _)| t))?;
        let baro = (0..2)
            .filter(|&i| !log.baro[i].is_empty())
            .map(|i| {
                let altitudes = log.baro[i]
                    .iter()
                    .map(|s| pressure_altitude_m(s.pressure_mbar));
                (i, log.baro[i].clone(), noise_sigma(altitudes, 0.01))
            })
            .collect::<Vec<_>>();
        let gnss = (0..2)
            .map(|i| {
                let fixes = log.gnss[i]
                    .iter()
                    .filter(|s| s.fix_ok && matches!(s.fix_type, 3 | 4))
                    .copied()
                    .collect::<Vec<_>>();
                (i, fixes)
            })
            .filter(|(_, fixes)| !fixes.is_empty())
            .collect::<Vec<_>>();
        baro.first()?;
        Some(Self {
            spline: Spline::new(start, end, Self::KNOT_S, MARGIN_S),
            accel_sigma_mps2: noise_sigma(accel.iter().map(|&(_, a)| a), 1e-3),
            accel,
            baro,
            gnss,
        })
    }

    fn accel_bias(&self) -> usize {
        self.spline.count()
    }

    /// Without GNSS nothing fixes absolute height, so the first barometer
    /// defines it and only the others get an offset.
    fn baro_offset(&self, slot: usize) -> Option<usize> {
        let first_free = usize::from(self.gnss.is_empty());
        (slot >= first_free).then(|| self.spline.count() + 1 + slot - first_free)
    }
}

impl Model for Vertical {
    fn unknowns(&self) -> usize {
        self.spline.count() + 1 + self.baro.len() - usize::from(self.gnss.is_empty())
    }

    fn latency_names(&self) -> Vec<String> {
        let baro = self
            .baro
            .iter()
            .map(|(i, _, _)| format!("BARO_BUS_{}", i + 1));
        let gnss = self.gnss.iter().map(|(i, _)| format!("GNSS_{i}"));
        baro.chain(gnss).collect()
    }

    fn rows(&self, latencies: &[f64]) -> Vec<Row> {
        let mut rows = Vec::new();
        for &(t, a_up) in &self.accel {
            let basis = self.spline.basis(t);
            let mut entries = scalar(&basis, &basis.d2, 1.0);
            entries.push((self.accel_bias(), 1.0));
            rows.push(Row {
                entries,
                rhs: a_up,
                sigma: self.accel_sigma_mps2,
                d_latency: None,
            });
        }
        for (slot, (_, samples, sigma)) in self.baro.iter().enumerate() {
            for sample in samples {
                let basis = self.spline.basis(sample.t - latencies[slot]);
                let mut entries = scalar(&basis, &basis.value, 1.0);
                entries.extend(self.baro_offset(slot).map(|offset| (offset, 1.0)));
                rows.push(Row {
                    entries,
                    rhs: pressure_altitude_m(sample.pressure_mbar),
                    sigma: *sigma,
                    d_latency: Some((slot, scalar(&basis, &basis.d1, -1.0))),
                });
            }
        }
        for (slot, (_, samples)) in self.gnss.iter().enumerate() {
            let parameter = self.baro.len() + slot;
            for sample in samples {
                let basis = self.spline.basis(sample.t - latencies[parameter]);
                rows.push(Row {
                    entries: scalar(&basis, &basis.value, 1.0),
                    rhs: sample.height_msl_m,
                    sigma: (sample.vertical_accuracy_mm / 1000.0).max(Self::MIN_SIGMA),
                    d_latency: Some((parameter, scalar(&basis, &basis.d1, -1.0))),
                });
                rows.push(Row {
                    entries: scalar(&basis, &basis.d1, 1.0),
                    rhs: -sample.velocity_down_mps,
                    sigma: sample.speed_accuracy_mps.max(Self::MIN_SIGMA),
                    d_latency: Some((parameter, scalar(&basis, &basis.d2, -1.0))),
                });
            }
        }
        rows
    }
}

/// Spline entries for a one-dimensional signal.
fn scalar(basis: &Basis, weights: &[f64; 4], scale: f64) -> Vec<(usize, f64)> {
    (0..4)
        .map(|k| (basis.first + k, scale * weights[k]))
        .collect()
}

/// Spline entries for one axis of a three-axis signal. Coefficients are
/// interleaved by axis so the normal equations stay banded.
fn on_axis(basis: &Basis, weights: &[f64; 4], axis: usize, scale: f64) -> Vec<(usize, f64)> {
    (0..4)
        .map(|k| (3 * (basis.first + k) + axis, scale * weights[k]))
        .collect()
}

fn span(times: impl Iterator<Item = f64>) -> Option<(f64, f64)> {
    times.fold(None, |range, t| match range {
        None => Some((t, t)),
        Some((start, end)) => Some((f64::min(start, t), f64::max(end, t))),
    })
}

/// Rotates `v` by the unit quaternion `[w, x, y, z]`.
fn rotate([w, x, y, z]: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let u = [x, y, z];
    let cross = |a: [f64; 3], b: [f64; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let t = cross(u, v).map(|c| 2.0 * c);
    let ut = cross(u, t);
    std::array::from_fn(|k| v[k] + w * t[k] + ut[k])
}
