//! IMU calibration: latency, LSM6DSO32 units, axes, and gyro bias.

use core::fmt;

use embassy_time::{Duration, Instant, with_deadline};
use lsm6dso32::{Acceleration, AngularRate};
use serde::{Deserialize, Serialize};

use super::{Calibrations, Name};
use crate::sensors::{IMU_0, IMU_1, IMU_COUNT, ImuId};
use crate::signals::IMU_CHANNEL;
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
    let (accel, gyro) = board_frame(raw);
    let bias = cal.correction.gyro_bias_dps;
    ImuSample {
        src: raw.src,
        ts: cal.sample_time(raw.ts),
        accel: Acceleration {
            x: accel[0],
            y: accel[1],
            z: accel[2],
        },
        gyro: AngularRate {
            x: gyro[0] - bias[0],
            y: gyro[1] - bias[1],
            z: gyro[2] - bias[2],
        },
    }
}

/// Acceleration in g and angular rate in deg/s, board frame, before the
/// per-unit correction.
fn board_frame(raw: RawImuSample) -> ([f32; 3], [f32; 3]) {
    let accel = Acceleration::from_raw(raw.accel, ACCEL_FULL_SCALE);
    let gyro = AngularRate::from_raw(raw.gyro, GYRO_FULL_SCALE);
    (
        sensor_to_board([accel.x, accel.y, accel.z]),
        sensor_to_board([gyro.x, gyro.y, gyro.z]),
    )
}

const CALIBRATION_TIME: Duration = Duration::from_secs(5);
const MIN_SAMPLES: u32 = 1_000;
const MAX_GYRO_NOISE_DPS: f32 = 0.57;
const MAX_ACCEL_NOISE_G: f32 = 0.03;

#[derive(Default)]
pub struct ImuCal {
    windows: [ImuWindow; IMU_COUNT],
}

impl ImuCal {
    pub async fn collect(&mut self) {
        let mut samples = IMU_CHANNEL
            .subscriber()
            .expect("IMU calibration subscriber slot must be free");
        let deadline = Instant::now() + CALIBRATION_TIME;
        while let Ok(reading) = with_deadline(deadline, samples.next_message_pure()).await {
            self.windows[reading.raw.src.index()].record(reading.raw);
        }
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
                let gyro_bias_dps = summary.gyro_mean_dps;
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

/// Statistics over a stationary window, in g and deg/s. The mean angular rate
/// is the gyro bias; the noise terms reject a window in which the board moved.
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
    gravity_error_noise_g: f32,
    gyro_mean_dps: [f32; 3],
    gyro_noise_dps: f32,
}

impl ImuWindow {
    fn record(&mut self, raw: RawImuSample) {
        let (accel, gyro) = board_frame(raw);
        let gravity_error = libm::sqrtf(accel.iter().map(|a| a * a).sum()) - 1.0;
        self.samples += 1;
        self.gravity_error_sum += gravity_error;
        self.gravity_error_square_sum += gravity_error * gravity_error;
        for (sum, rate) in self.gyro_sum.iter_mut().zip(gyro) {
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
            gravity_error_noise_g: libm::sqrtf(
                (self.gravity_error_square_sum / count - gravity_error_mean * gravity_error_mean)
                    .max(0.0),
            ),
            gyro_mean_dps: gyro_mean,
            gyro_noise_dps: libm::sqrtf((self.gyro_square_sum / count - gyro_mean_square).max(0.0)),
        })
    }
}

fn valid_summary(s: ImuWindowSummary) -> bool {
    s.samples >= MIN_SAMPLES
        && s.gyro_noise_dps.is_finite()
        && s.gyro_noise_dps <= MAX_GYRO_NOISE_DPS
        && s.gravity_error_noise_g.is_finite()
        && s.gravity_error_noise_g <= MAX_ACCEL_NOISE_G
        && s.gyro_mean_dps.iter().all(|value| value.is_finite())
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
                "{}: rejected ({} samples, gyro noise {:.3} deg/s, accel noise {:.4} g)",
                self.id.name(),
                s.samples,
                s.gyro_noise_dps,
                s.gravity_error_noise_g
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
