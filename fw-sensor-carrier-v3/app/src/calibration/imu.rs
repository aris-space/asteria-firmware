//! IMU calibration: latency, LSM6DSO32 units, axes, and gyro bias.

use core::fmt;

use asteria_sef_light::{ImuMeasurement, STANDARD_GRAVITY_MPS2};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, with_timeout};
use lsm6dso32::{Acceleration, AngularRate};
use serde::{Deserialize, Serialize};

use super::{Calibrations, Name};
use crate::sef::imu_measurement;
use crate::sensors::{IMU_0, IMU_1, IMU_COUNT, ImuId};
use crate::signals::IMU_CHANNELS;
use crate::storage::Storage;
use crate::tasks::readout::imu::{ACCEL_FULL_SCALE, GYRO_FULL_SCALE};
use crate::types::{ImuSample, RawImuSample};

/// Sensor-to-board axis remap for the LSM6DSO32 on this board: negate x and z.
fn sensor_to_board([x, y, z]: [f32; 3]) -> [f32; 3] {
    [-x, y, -z]
}

/// Gyro offsets in board axes and degrees per second. A stationary
/// calibration cannot determine the three accelerometer offsets independently.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Correction {
    gyro_bias_dps: [f32; 3],
    samples: u32,
}

impl super::Correction for Correction {
    const DEFAULT: Self = Self {
        gyro_bias_dps: [0.0; 3],
        samples: 0,
    };

    fn is_valid(&self) -> bool {
        self.gyro_bias_dps.iter().all(|v| v.is_finite())
    }
}

impl fmt::Display for Correction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let b = self.gyro_bias_dps;
        write!(
            f,
            "  gyro bias [{:.4}, {:.4}, {:.4}] deg/s ({} samples)",
            b[0], b[1], b[2], self.samples
        )
    }
}

pub static CAL: Calibrations<ImuId, Correction, IMU_COUNT> = Calibrations::new(ImuId::ALL);

pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    let cal = CAL.applied(raw.src);
    let accel = Acceleration::from_raw(raw.accel, ACCEL_FULL_SCALE);
    let gyro = AngularRate::from_raw(raw.gyro, GYRO_FULL_SCALE);
    let [ax, ay, az] = sensor_to_board([accel.x, accel.y, accel.z]);
    let [gx, gy, gz] = sensor_to_board([gyro.x, gyro.y, gyro.z]);
    let bias = cal.correction.gyro_bias_dps;
    ImuSample {
        src: raw.src,
        ts: cal.sample_time(raw.ts),
        accel: Acceleration {
            x: ax,
            y: ay,
            z: az,
        },
        gyro: AngularRate {
            x: gx - bias[0],
            y: gy - bias[1],
            z: gz - bias[2],
        },
    }
}

const CALIBRATION_TIME: Duration = Duration::from_secs(5);
const MIN_SAMPLES: u32 = 1_000;
const MAX_GYRO_NOISE_RAD_S: f32 = 0.01;
const MAX_ACCEL_NOISE_MPS2: f32 = 0.3;

/// Stationary gyro calibration: [`collect`](Self::collect) records both IMUs
/// for [`CALIBRATION_TIME`], then [`finish`](Self::finish) stores the result.
#[derive(Default)]
pub struct ImuCal {
    windows: [ImuWindow; IMU_COUNT],
}

impl ImuCal {
    pub async fn collect(&mut self) {
        let mut imu_0 = IMU_CHANNELS[IMU_0.index()]
            .subscriber()
            .expect("second IMU subscriber unavailable");
        let mut imu_1 = IMU_CHANNELS[IMU_1.index()]
            .subscriber()
            .expect("second IMU subscriber unavailable");
        let deadline = Instant::now() + CALIBRATION_TIME;
        while Instant::now() < deadline {
            // Drain one sample per source before waiting, so FIFO bursts from
            // either IMU cannot starve the other calibration window.
            let first = imu_0.try_next_message_pure();
            let second = imu_1.try_next_message_pure();
            if first.is_some() || second.is_some() {
                for sample in first.into_iter().chain(second) {
                    self.record(sample);
                }
                continue;
            }
            let remaining = deadline - Instant::now();
            let next = select(imu_0.next_message_pure(), imu_1.next_message_pure());
            let sample = match with_timeout(remaining, next).await {
                Ok(Either::First(sample)) | Ok(Either::Second(sample)) => sample,
                Err(_) => break,
            };
            self.record(sample);
        }
    }

    fn record(&mut self, sample: ImuSample) {
        self.windows[sample.src.index()].record(imu_measurement(
            [sample.accel.x, sample.accel.y, sample.accel.z],
            [sample.gyro.x, sample.gyro.y, sample.gyro.z],
        ));
    }

    pub fn counts(&self) -> [u32; IMU_COUNT] {
        self.windows.each_ref().map(|window| window.samples)
    }

    pub async fn finish(self, name: &str, storage: &Storage) -> [CalReport; IMU_COUNT] {
        let summaries = self.windows.map(ImuWindow::summary);
        let valid = summaries
            .iter()
            .all(|summary| summary.is_some_and(valid_summary));
        let mut reports = [
            CalReport::new(IMU_0, summaries[0]),
            CalReport::new(IMU_1, summaries[1]),
        ];
        for report in &mut reports {
            report.all_valid = valid;
        }
        if valid {
            for report in &mut reports {
                let summary = report.summary.expect("validated IMU summary");
                let old = CAL.applied(report.id).correction.gyro_bias_dps;
                const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;
                let gyro_bias_dps = core::array::from_fn(|axis| {
                    old[axis] + summary.gyro_mean_rad_s[axis] * RAD_TO_DEG
                });
                let correction = Correction {
                    gyro_bias_dps,
                    samples: summary.samples,
                };
                report.stored = CAL
                    .store_correction(storage, report.id, Name::new(name), correction)
                    .await;
                report.bias_dps = gyro_bias_dps;
            }
        }
        reports
    }
}

/// Statistics over a stationary window. The mean angular rate is the gyro
/// bias; the noise terms reject a window in which the board moved.
#[derive(Clone, Copy, Default)]
struct ImuWindow {
    samples: u32,
    gravity_error_sum: f32,
    gravity_error_square_sum: f32,
    gyro_sum: [f32; 3],
    gyro_square_sum: f32,
}

#[derive(Clone, Copy)]
struct ImuWindowSummary {
    samples: u32,
    gravity_error_noise_mps2: f32,
    gyro_mean_rad_s: [f32; 3],
    gyro_noise_rad_s: f32,
}

impl ImuWindow {
    fn record(&mut self, measurement: ImuMeasurement) {
        let acceleration = libm::sqrtf(
            measurement
                .acceleration_body_mps2
                .iter()
                .map(|value| value * value)
                .sum(),
        );
        let gravity_error = acceleration - STANDARD_GRAVITY_MPS2;
        self.samples += 1;
        self.gravity_error_sum += gravity_error;
        self.gravity_error_square_sum += gravity_error * gravity_error;
        for (sum, rate) in self
            .gyro_sum
            .iter_mut()
            .zip(measurement.angular_rate_body_rad_s)
        {
            *sum += rate;
            self.gyro_square_sum += rate * rate;
        }
    }

    fn summary(self) -> Option<ImuWindowSummary> {
        if self.samples == 0 {
            return None;
        }
        let count = self.samples as f32;
        let gravity_error_mean = self.gravity_error_sum / count;
        let gyro_mean = self.gyro_sum.map(|sum| sum / count);
        let gyro_mean_square = gyro_mean.iter().map(|rate| rate * rate).sum::<f32>();
        Some(ImuWindowSummary {
            samples: self.samples,
            gravity_error_noise_mps2: libm::sqrtf(
                (self.gravity_error_square_sum / count - gravity_error_mean * gravity_error_mean)
                    .max(0.0),
            ),
            gyro_mean_rad_s: gyro_mean,
            gyro_noise_rad_s: libm::sqrtf(
                (self.gyro_square_sum / count - gyro_mean_square).max(0.0),
            ),
        })
    }
}

fn valid_summary(s: ImuWindowSummary) -> bool {
    s.samples >= MIN_SAMPLES
        && s.gyro_noise_rad_s.is_finite()
        && s.gyro_noise_rad_s <= MAX_GYRO_NOISE_RAD_S
        && s.gravity_error_noise_mps2.is_finite()
        && s.gravity_error_noise_mps2 <= MAX_ACCEL_NOISE_MPS2
        && s.gyro_mean_rad_s.iter().all(|value| value.is_finite())
}

pub struct CalReport {
    id: ImuId,
    summary: Option<ImuWindowSummary>,
    bias_dps: [f32; 3],
    all_valid: bool,
    stored: bool,
}

impl CalReport {
    fn new(id: ImuId, summary: Option<ImuWindowSummary>) -> Self {
        Self {
            id,
            summary,
            bias_dps: [0.0; 3],
            all_valid: false,
            stored: false,
        }
    }

    pub fn stored(&self) -> bool {
        self.stored
    }
}

impl fmt::Display for CalReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let Some(s) = self.summary else {
            return write!(f, "{}: no samples", self.id.name());
        };
        if !valid_summary(s) {
            return write!(
                f,
                "{}: rejected ({} samples, gyro noise {:.4} rad/s, accel noise {:.3} m/s²)",
                self.id.name(),
                s.samples,
                s.gyro_noise_rad_s,
                s.gravity_error_noise_mps2
            );
        }
        if !self.all_valid {
            return write!(
                f,
                "{}: skipped because the other IMU was unstable",
                self.id.name()
            );
        }
        let b = self.bias_dps;
        write!(
            f,
            "{}: gyro bias [{:.4}, {:.4}, {:.4}] deg/s ({} samples) {}",
            self.id.name(),
            b[0],
            b[1],
            b[2],
            s.samples,
            if self.stored {
                "stored"
            } else {
                "flash write failed"
            }
        )
    }
}
