//! Synthetic SD logs with known latencies, in the firmware's CSV format.
//!
//! A scenario defines the board's upward acceleration and body rotation rate.
//! Its motion is integrated on a fine grid, then every sensor is sampled at its
//! real rate with noise, biases, timestamp jitter, and the latencies in
//! [`TRUE_LATENCY_S`].

use std::f64::consts::TAU;
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use nalgebra::{UnitQuaternion, Vector3};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use rand_distr::{Distribution, Normal};

use crate::log::STANDARD_GRAVITY;

pub const TRUE_LATENCY_S: [(&str, f64); 7] = [
    ("IMU_1", 1.5e-3),
    ("MAG_BUS_1", 40e-3),
    ("MAG_BUS_2", 55e-3),
    ("BARO_BUS_1", 6e-3),
    ("BARO_BUS_2", 8e-3),
    ("GNSS_0", 45e-3),
    ("GNSS_1", 60e-3),
];

pub fn true_latency(name: &str) -> f64 {
    TRUE_LATENCY_S
        .iter()
        .find(|(sensor, _)| *sensor == name)
        .map_or(0.0, |&(_, latency)| latency)
}

pub struct Scenario {
    pub name: &'static str,
    pub description: &'static str,
    pub duration_s: f64,
    pub gnss_fix: bool,
    /// Upward acceleration in m/s² from time and upward velocity.
    up_accel: fn(f64, f64) -> f64,
    /// Body rotation rate in rad/s.
    body_rate: fn(f64) -> [f64; 3],
}

pub const SCENARIOS: [Scenario; 9] = [
    Scenario {
        name: "still",
        description: "board lying on a table",
        duration_s: 60.0,
        gnss_fix: true,
        up_accel: |_, _| 0.0,
        body_rate: |_| [0.0; 3],
    },
    Scenario {
        name: "tumble",
        description: "turned through all orientations by hand, ~100 dps",
        duration_s: 60.0,
        gnss_fix: false,
        up_accel: |_, _| 0.0,
        body_rate: tumble,
    },
    Scenario {
        name: "hand-lift",
        description: "raised and lowered ±0.4 m at 0.6 Hz, indoors",
        duration_s: 60.0,
        gnss_fix: false,
        up_accel: |t, _| sine_accel(0.4, 0.6, t),
        body_rate: wobble,
    },
    Scenario {
        name: "hand-shake",
        description: "shaken up and down ±0.3 m at 1.2 Hz, indoors",
        duration_s: 60.0,
        gnss_fix: false,
        up_accel: |t, _| sine_accel(0.3, 1.2, t),
        body_rate: wobble,
    },
    Scenario {
        name: "elevator",
        description: "elevator 30 m up and down at 1.5 m/s, indoors",
        duration_s: 80.0,
        gnss_fix: false,
        up_accel: |t, _| match t {
            t if (5.0..6.5).contains(&t) || (61.5..63.0).contains(&t) => 1.0,
            t if (26.5..28.0).contains(&t) || (40.0..41.5).contains(&t) => -1.0,
            _ => 0.0,
        },
        body_rate: |_| [0.0; 3],
    },
    Scenario {
        name: "walk-hill",
        description: "walking over a hill outdoors, ±0.4 m/s vertical",
        duration_s: 180.0,
        gnss_fix: true,
        up_accel: |t, _| 0.4 * TAU / 40.0 * (TAU * t / 40.0).cos() + sine_accel(0.03, 1.8, t),
        body_rate: |t| {
            [
                0.3 * (TAU * 0.9 * t).sin(),
                0.2 * (TAU * 1.8 * t).sin(),
                0.4 * (TAU * 0.1 * t).sin(),
            ]
        },
    },
    Scenario {
        name: "car-hills",
        description: "driving over hills, ±2.5 m/s vertical with road bumps",
        duration_s: 180.0,
        gnss_fix: true,
        up_accel: |t, _| 2.5 * TAU / 30.0 * (TAU * t / 30.0).cos() + 0.5 * (TAU * 3.0 * t).sin(),
        body_rate: |t| {
            [
                0.05 * (TAU * 0.7 * t).sin(),
                0.05 * (TAU * 0.5 * t).sin(),
                0.2 * (TAU * t / 20.0).sin(),
            ]
        },
    },
    Scenario {
        name: "flight",
        description: "20 s on the pad, 3 s boost at 6 g, coast to ~1.6 km, descent at 20 m/s",
        duration_s: 110.0,
        gnss_fix: true,
        up_accel: |t, v| match t {
            t if t < 20.0 => 0.0,
            t if t < 23.0 => 60.0,
            _ if v > 0.0 => -STANDARD_GRAVITY - 4e-4 * v * v,
            _ => -STANDARD_GRAVITY + 0.0245 * v * v,
        },
        body_rate: |t| match t {
            t if t < 20.0 => [0.0; 3],
            t if t < 38.0 => [
                3.0,
                0.05 * (TAU * 0.5 * t).sin(),
                0.05 * (TAU * 0.5 * t).cos(),
            ],
            _ => [
                0.3 * (TAU * 0.2 * t).sin(),
                0.3 * (TAU * 0.15 * t).cos(),
                0.5,
            ],
        },
    },
    Scenario {
        name: "bench-session",
        description: "30 s tumble, 10 s still, 30 s shake, 20 s still, indoors",
        duration_s: 90.0,
        gnss_fix: false,
        up_accel: |t, _| {
            if (40.0..70.0).contains(&t) {
                sine_accel(0.3, 1.2, t)
            } else {
                0.0
            }
        },
        body_rate: |t| match t {
            t if t < 30.0 => tumble(t),
            t if (40.0..70.0).contains(&t) => wobble(t),
            _ => [0.0; 3],
        },
    },
];

fn tumble(t: f64) -> [f64; 3] {
    [
        1.6 * (TAU * 0.31 * t).sin(),
        1.3 * (TAU * 0.23 * t + 1.0).sin(),
        1.9 * (TAU * 0.17 * t + 2.0).sin(),
    ]
}

fn wobble(t: f64) -> [f64; 3] {
    [
        0.15 * (TAU * 0.7 * t).sin(),
        0.1 * (TAU * 0.5 * t + 1.0).sin(),
        0.1 * (TAU * 0.3 * t).sin(),
    ]
}

/// Upward acceleration of `amplitude·sin(2π·f·t)` metres of height.
fn sine_accel(amplitude: f64, frequency: f64, t: f64) -> f64 {
    -amplitude * (TAU * frequency).powi(2) * (TAU * frequency * t).sin()
}

/// The true motion on a fine grid.
struct Truth {
    dt: f64,
    height: Vec<f64>,
    velocity: Vec<f64>,
    accel: Vec<f64>,
    attitude: Vec<UnitQuaternion<f64>>,
    rate: Vec<[f64; 3]>,
}

impl Truth {
    const DT: f64 = 2e-4;

    fn new(scenario: &Scenario) -> Self {
        let steps = (scenario.duration_s / Self::DT) as usize + 2;
        let mut truth = Truth {
            dt: Self::DT,
            height: Vec::with_capacity(steps),
            velocity: Vec::with_capacity(steps),
            accel: Vec::with_capacity(steps),
            attitude: Vec::with_capacity(steps),
            rate: Vec::with_capacity(steps),
        };
        let (mut h, mut v) = (0.0, 0.0);
        let mut q = UnitQuaternion::identity();
        for k in 0..steps {
            let t = k as f64 * Self::DT;
            let a = (scenario.up_accel)(t, v);
            let w = (scenario.body_rate)(t);
            truth.height.push(h);
            truth.velocity.push(v);
            truth.accel.push(a);
            truth.attitude.push(q);
            truth.rate.push(w);
            v += a * Self::DT;
            h += v * Self::DT;
            q *= UnitQuaternion::from_scaled_axis(Vector3::from(w) * Self::DT);
        }
        truth
    }

    /// Index and fraction of `t` between grid points.
    fn locate(&self, t: f64) -> (usize, f64) {
        let u = (t / self.dt).clamp(0.0, (self.height.len() - 2) as f64);
        (u as usize, u.fract())
    }

    fn scalar(&self, values: &[f64], t: f64) -> f64 {
        let (i, u) = self.locate(t);
        values[i] + u * (values[i + 1] - values[i])
    }

    fn attitude(&self, t: f64) -> UnitQuaternion<f64> {
        let (i, u) = self.locate(t);
        self.attitude[i].nlerp(&self.attitude[i + 1], u)
    }

    fn rate(&self, t: f64) -> Vector3<f64> {
        let (i, u) = self.locate(t);
        let (a, b) = (self.rate[i], self.rate[i + 1]);
        Vector3::from(std::array::from_fn(|k| a[k] + u * (b[k] - a[k])))
    }
}

struct Noise(StdRng);

impl Noise {
    fn normal(&mut self, sigma: f64) -> f64 {
        Normal::new(0.0, sigma).unwrap().sample(&mut self.0)
    }

    fn vector(&mut self, sigma: f64) -> Vector3<f64> {
        Vector3::new(self.normal(sigma), self.normal(sigma), self.normal(sigma))
    }

    fn uniform(&mut self, max: f64) -> f64 {
        self.0.random_range(0.0..max)
    }
}

/// Writes one simulated `LOGnnnn`-style directory.
pub fn write(scenario: &Scenario, seed: u64, dir: &Path) -> std::io::Result<()> {
    fs::create_dir_all(dir)?;
    let truth = Truth::new(scenario);
    let mut noise = Noise(StdRng::seed_from_u64(seed));
    let duration = scenario.duration_s;
    let us = |t: f64| (t * 1e6).round().max(0.0) as u64;

    let mut csv = String::from(
        "read_us,raw_us,cal_us,imu,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n",
    );
    let accel_bias = [
        Vector3::new(0.002, -0.003, 0.005),
        Vector3::new(-0.004, 0.001, -0.002),
    ];
    let gyro_bias = [
        Vector3::new(0.03, -0.05, 0.02),
        Vector3::new(-0.02, 0.04, 0.05),
    ];
    for imu in 0..2 {
        let latency = if imu == 0 { 0.0 } else { true_latency("IMU_1") };
        for k in 0.. {
            let t = k as f64 / 833.0 + imu as f64 * 3e-4;
            if t > duration {
                break;
            }
            let a_ned = Vector3::new(0.0, 0.0, -truth.scalar(&truth.accel, t));
            let gravity = Vector3::new(0.0, 0.0, STANDARD_GRAVITY);
            let accel_g = truth.attitude(t).inverse() * (a_ned - gravity) / STANDARD_GRAVITY
                + accel_bias[imu]
                + noise.vector(0.004);
            let gyro_dps = truth.rate(t).map(f64::to_degrees) + gyro_bias[imu] + noise.vector(0.07);
            let stamp = us(t + latency + noise.normal(2e-5));
            writeln!(
                csv,
                "{stamp},{stamp},{stamp},{imu},0,0,0,0,0,0,{:.6},{:.6},{:.6},{:.4},{:.4},{:.4}",
                accel_g.x, accel_g.y, accel_g.z, gyro_dps.x, gyro_dps.y, gyro_dps.z
            )
            .unwrap();
        }
    }
    fs::write(dir.join("IMU.CSV"), csv)?;

    let earth_field = Vector3::new(20_000.0, 1_000.0, 45_000.0);
    let hard_iron = Vector3::new(800.0, -400.0, 300.0);
    let soft_iron = Vector3::new(1.02, 0.98, 1.0);
    let mut csv = String::from("read_us,raw_us,cal_us,mag,x_raw,y_raw,z_raw,x_nt,y_nt,z_nt\n");
    for (mag, name) in ["MAG_BUS_1", "MAG_BUS_2"].into_iter().enumerate() {
        for k in 0.. {
            let t = k as f64 * 0.1 + mag as f64 * 0.05;
            if t > duration {
                break;
            }
            let field = (truth.attitude(t).inverse() * earth_field).component_mul(&soft_iron)
                + hard_iron
                + noise.vector(150.0);
            let stamp = us(t + true_latency(name));
            writeln!(
                csv,
                "{stamp},{stamp},{stamp},{mag},0,0,0,{:.2},{:.2},{:.2}",
                field.x, field.y, field.z
            )
            .unwrap();
        }
    }
    fs::write(dir.join("MAG.CSV"), csv)?;

    let mut csv = String::from("read_us,raw_us,cal_us,baro,pressure_mbar,temperature_c\n");
    for (baro, (name, base)) in [("BARO_BUS_1", 300.0), ("BARO_BUS_2", 290.0)]
        .into_iter()
        .enumerate()
    {
        for k in 0.. {
            let t = k as f64 / 40.0 + baro as f64 * 0.01;
            if t > duration {
                break;
            }
            let altitude = base + truth.scalar(&truth.height, t) + noise.normal(0.25);
            let pressure = 1013.25 * (1.0 - altitude / 44_330.0).powf(1.0 / 0.190_294_95);
            let stamp = us(t + true_latency(name) + noise.normal(5e-4));
            writeln!(csv, "{stamp},{stamp},{stamp},{baro},{pressure:.4},20.00").unwrap();
        }
    }
    fs::write(dir.join("BARO.CSV"), csv)?;

    let mut csv = String::from(
        "read_us,raw_us,cal_us,gnss,itow_ms,num_satellites,fix_type,fix_ok,latitude_deg,longitude_deg,height_msl_m,velocity_down_mps,horizontal_accuracy_mm,vertical_accuracy_mm,speed_accuracy_mps,pdop_centi\n",
    );
    for (gnss, name) in ["GNSS_0", "GNSS_1"].into_iter().enumerate() {
        let mut wander = 0.0;
        for k in 0.. {
            let t = k as f64 * 0.05;
            if t > duration {
                break;
            }
            wander += noise.normal(0.02);
            let itow = 100_000 + 50 * k;
            // UART transfer and parsing add up to 2 ms after the latency.
            let stamp = us(t + true_latency(name) + noise.uniform(2e-3));
            if scenario.gnss_fix {
                let height = 420.0 + truth.scalar(&truth.height, t) + wander + noise.normal(0.3);
                let down = -truth.scalar(&truth.velocity, t) + noise.normal(0.05);
                writeln!(
                    csv,
                    "{stamp},{stamp},{stamp},{gnss},{itow},14,3,1,47.00000000,8.00000000,{height:.3},{down:.4},900,1500,0.1000,140"
                )
                .unwrap();
            } else {
                writeln!(
                    csv,
                    "{stamp},{stamp},{stamp},{gnss},{itow},0,0,0,0.00000000,0.00000000,0.000,0.0000,0,0,0.0000,9999"
                )
                .unwrap();
            }
        }
    }
    fs::write(dir.join("GNSS.CSV"), csv)?;

    // The estimator's attitude is slightly off from the truth.
    let attitude_error = UnitQuaternion::from_euler_angles(0.008, -0.005, 0.02);
    let mut csv = String::from(
        "cal_us,imu,selected,msl_ready,redundancy_ready,height_msl_m,velocity_mps,bias0_m,bias1_m,height_std_m,velocity_std_mps,bias0_std_m,bias1_std_m,score,qw,qx,qy,qz\n",
    );
    for k in 0.. {
        let t = k as f64 * 0.05;
        if t > duration {
            break;
        }
        let q = truth.attitude(t) * attitude_error;
        for imu in 0..2 {
            writeln!(
                csv,
                "{},{imu},{},1,1,0,0,0,0,0,0,0,0,0,{:.6},{:.6},{:.6},{:.6}",
                us(t),
                u8::from(imu == 0),
                q.w,
                q.i,
                q.j,
                q.k
            )
            .unwrap();
        }
    }
    fs::write(dir.join("STATE.CSV"), csv)?;
    fs::write(
        dir.join("DHT.CSV"),
        "read_us,raw_us,cal_us,dht,temperature_c,humidity_rh\n",
    )?;
    fs::write(
        dir.join("DROPS.CSV"),
        "uptime_us,state,imu,mag,gnss,baro,dht\n",
    )?;
    Ok(())
}
