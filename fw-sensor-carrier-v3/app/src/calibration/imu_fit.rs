//! Two-IMU alignment estimator: pure math, no I/O.
//!
//! Feed board-frame accelerometer and gyro samples for IMU 0 and IMU 1 across
//! several still poses. [`Estimator`] gates quasi-static gravity readings,
//! accumulates a Kabsch cross-covariance, and [`solve`](Estimator::solve)s the
//! relative rotation (IMU 1 -> IMU 0) plus each gyro's zero-rate bias. Depends
//! only on `nalgebra` and `libm`, so it could be lifted into its own crate.

use nalgebra::{Matrix3, Rotation3, Vector3};

/// Number of IMUs the cross-calibration relates.
pub const IMUS: usize = 2;

// Accept a sample as gravity only when |accel| is near 1 g and the gyro shows
// the board is still: while moving, the two IMUs (at different spots on the
// board) read different acceleration that no single rotation can reconcile.
const GRAVITY_LO_G: f32 = 0.95;
const GRAVITY_HI_G: f32 = 1.05;
const QUASI_STATIC_DPS: f32 = 4.0;

/// The relative rotation between the two IMUs and how well it is determined.
pub struct RotationFit {
    /// IMU 1 -> IMU 0 alignment: `accel0 ~= rotation * accel1`.
    pub rotation: Matrix3<f32>,
    /// Rotation axis (unit), or `None` when the angle is ~0 and undefined.
    pub axis: Option<Vector3<f32>>,
    pub misalign_deg: f32,
    pub residual_deg: f32,
    /// Smallest/largest gravity-spread singular value: 1.0 = poses covered all
    /// axes, near 0 = they stayed in roughly one plane.
    pub coverage: f32,
}

/// Everything the estimator produces from the captured poses.
pub struct Fit {
    pub rotation: RotationFit,
    pub pairs: usize,
    /// Per-IMU gyro zero-rate bias (board frame, dps): the mean still reading.
    pub gyro_bias: [Vector3<f32>; IMUS],
    /// Per-IMU mean |accel| over the still samples, in g.
    pub accel_g: [f32; IMUS],
    pub gravity_n: [usize; IMUS],
    pub gyro_n: [usize; IMUS],
    pub peak_dps: f32,
}

#[derive(Default)]
pub struct Estimator {
    // Gravity cross-covariance `sum(a1 * a0^T)` for the Kabsch fit.
    h: Matrix3<f32>,
    pairs: usize,
    accel_mag_sum: [f32; IMUS],
    gravity_n: [usize; IMUS],
    gyro_sum: [Vector3<f32>; IMUS],
    gyro_n: [usize; IMUS],
    peak_dps: f32,
    // Latest accepted gravity unit vector from each IMU in the current pose.
    latch: [Option<Vector3<f32>>; IMUS],
}

impl Estimator {
    /// Start a new still pose, clearing the within-pose pairing latch so the
    /// first reading of a pose isn't paired against the previous orientation.
    pub fn begin_pose(&mut self) {
        self.latch = [None; IMUS];
    }

    /// Feed one board-frame sample for IMU `idx`. Accepted as gravity only when
    /// the board is still (gyro near zero) and |accel| ~ 1 g; returns whether it
    /// was accepted.
    pub fn observe(&mut self, idx: usize, accel: Vector3<f32>, gyro: Vector3<f32>) -> bool {
        self.peak_dps = self.peak_dps.max(gyro.norm());
        if gyro.norm() > QUASI_STATIC_DPS {
            return false;
        }
        self.gyro_sum[idx] += gyro;
        self.gyro_n[idx] += 1;
        let n = accel.norm();
        if !(GRAVITY_LO_G..=GRAVITY_HI_G).contains(&n) {
            return false;
        }
        let u = accel / n;
        self.accel_mag_sum[idx] += n;
        self.gravity_n[idx] += 1;
        self.latch[idx] = Some(u);
        if let Some(other) = self.latch[idx ^ 1] {
            // H = sum(a1 * a0^T): order the outer product by which IMU is which.
            self.h += if idx == 0 {
                other * u.transpose()
            } else {
                u * other.transpose()
            };
            self.pairs += 1;
        }
        true
    }

    pub fn solve(&self) -> Fit {
        Fit {
            rotation: self.rotation_fit(),
            pairs: self.pairs,
            gyro_bias: [
                mean(self.gyro_sum[0], self.gyro_n[0]),
                mean(self.gyro_sum[1], self.gyro_n[1]),
            ],
            accel_g: [
                mean_mag(self.accel_mag_sum[0], self.gravity_n[0]),
                mean_mag(self.accel_mag_sum[1], self.gravity_n[1]),
            ],
            gravity_n: self.gravity_n,
            gyro_n: self.gyro_n,
            peak_dps: self.peak_dps,
        }
    }

    fn rotation_fit(&self) -> RotationFit {
        if self.pairs == 0 {
            return RotationFit {
                rotation: Matrix3::identity(),
                axis: None,
                misalign_deg: 0.0,
                residual_deg: f32::NAN,
                coverage: 0.0,
            };
        }
        let svd = self.h.svd(true, true);
        let u = svd.u.unwrap();
        let v_t = svd.v_t.unwrap();
        let mut d = Matrix3::identity();
        d[(2, 2)] = (u * v_t).determinant().signum();
        let rotation = v_t.transpose() * d * u.transpose();
        let rot = Rotation3::from_matrix_unchecked(rotation);
        // sum(a0 . R a1) = trace(R H); divide by count for the mean cosine.
        let mean_cos = ((rotation * self.h).trace() / self.pairs as f32).clamp(-1.0, 1.0);
        let sv = svd.singular_values;
        RotationFit {
            rotation,
            axis: rot.axis().map(|a| a.into_inner()),
            misalign_deg: rot.angle().to_degrees(),
            residual_deg: libm::acosf(mean_cos).to_degrees(),
            coverage: if sv[0] > 0.0 { sv[2] / sv[0] } else { 0.0 },
        }
    }
}

fn mean(sum: Vector3<f32>, n: usize) -> Vector3<f32> {
    if n == 0 {
        Vector3::zeros()
    } else {
        sum / n as f32
    }
}

fn mean_mag(sum: f32, n: usize) -> f32 {
    if n == 0 { f32::NAN } else { sum / n as f32 }
}
