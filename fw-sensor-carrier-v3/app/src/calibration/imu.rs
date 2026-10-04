use core::fmt;

use defmt::{info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::once_lock::OnceLock;
use embassy_time::Duration;
use lsm6dso32::{Acceleration, AngularRate};
use serde::{Deserialize, Serialize};

use super::Name;
use crate::sensors::{IMU_0, IMU_1, IMU_COUNT, ImuId};
use crate::signals::IMU_CHANNELS;
use crate::storage::Storage;
use crate::tasks::readout::imu::{ACCEL_FULL_SCALE, GYRO_FULL_SCALE};
use crate::types::{ImuSample, RawImuSample};
use fw_sensor_carrier_v3::sef::{ImuWindow, ImuWindowSummary, imu_measurement};

static CAL: OnceLock<[StoredCal; IMU_COUNT]> = OnceLock::new();
const DEFAULTS: [StoredCal; IMU_COUNT] = [StoredCal::DEFAULT; IMU_COUNT];
const CALIBRATION_TIME: Duration = Duration::from_secs(5);
const MIN_SAMPLES: u32 = 1_000;
const MAX_GYRO_NOISE_RAD_S: f32 = 0.01;
const MAX_ACCEL_NOISE_MPS2: f32 = 0.3;

// Measurement latency has not been measured, so retain the read-completion
// timestamp rather than applying an assumed offset.
pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    let accel = Acceleration::from_raw(raw.accel, ACCEL_FULL_SCALE);
    let gyro = AngularRate::from_raw(raw.gyro, GYRO_FULL_SCALE);
    let [ax, ay, az] = sensor_to_board([accel.x, accel.y, accel.z]);
    let [gx, gy, gz] = sensor_to_board([gyro.x, gyro.y, gyro.z]);
    let bias = CAL.try_get().unwrap_or(&DEFAULTS)[raw.src.index()].gyro_bias_dps;
    ImuSample {
        src: raw.src,
        ts: raw.ts,
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

/// Gyro offsets are in board axes and degrees per second. A stationary
/// calibration cannot determine the three accelerometer offsets independently.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct StoredCal {
    pub name: Name,
    gyro_bias_dps: [f32; 3],
    samples: u32,
}

impl StoredCal {
    const DEFAULT: Self = Self {
        name: Name::new("default"),
        gyro_bias_dps: [0.0; 3],
        samples: 0,
    };

    pub fn differs_from(&self, applied: &Self) -> bool {
        self.name != applied.name || self.gyro_bias_dps != applied.gyro_bias_dps
    }
}

impl fmt::Display for StoredCal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let b = self.gyro_bias_dps;
        write!(
            f,
            "\"{}\" gyro bias [{:.4}, {:.4}, {:.4}] deg/s ({} samples)",
            self.name, b[0], b[1], b[2], self.samples
        )
    }
}

pub async fn load(storage: &Storage) {
    let cal = [
        load_one(storage, IMU_0).await,
        load_one(storage, IMU_1).await,
    ];
    let _ = CAL.init(cal);
}

async fn load_one(storage: &Storage, id: ImuId) -> StoredCal {
    match storage.load::<StoredCal>(&id.key()).await {
        Some(cal) if cal.gyro_bias_dps.iter().all(|v| v.is_finite()) => {
            info!(
                "{}: gyro cal \"{}\" loaded from flash",
                id,
                cal.name.as_str()
            );
            cal
        }
        _ => {
            warn!("{}: no valid gyro cal in flash, using zero offset", id);
            StoredCal::DEFAULT
        }
    }
}

pub fn applied() -> [StoredCal; IMU_COUNT] {
    *CAL.try_get().unwrap_or(&DEFAULTS)
}

pub async fn stored(storage: &Storage) -> [Option<StoredCal>; IMU_COUNT] {
    [
        storage.load::<StoredCal>(&IMU_0.key()).await,
        storage.load::<StoredCal>(&IMU_1.key()).await,
    ]
}

pub struct ImuCal {
    windows: [ImuWindow; IMU_COUNT],
}

impl ImuCal {
    pub fn new() -> Self {
        Self {
            windows: [ImuWindow::default(); IMU_COUNT],
        }
    }

    pub async fn collect(&mut self) {
        let mut imu_0 = IMU_CHANNELS[IMU_0.index()]
            .subscriber()
            .expect("second IMU subscriber unavailable");
        let mut imu_1 = IMU_CHANNELS[IMU_1.index()]
            .subscriber()
            .expect("second IMU subscriber unavailable");
        let deadline = embassy_time::Instant::now() + CALIBRATION_TIME;
        while embassy_time::Instant::now() < deadline {
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
            let remaining = deadline - embassy_time::Instant::now();
            let next = select(imu_0.next_message_pure(), imu_1.next_message_pure());
            let sample = match embassy_time::with_timeout(remaining, next).await {
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
        self.windows.each_ref().map(|window| window.samples())
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
                let old = applied()[report.id.index()].gyro_bias_dps;
                const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;
                let gyro_bias_dps = core::array::from_fn(|axis| {
                    old[axis] + summary.gyro_mean_rad_s[axis] * RAD_TO_DEG
                });
                let cal = StoredCal {
                    name: Name::new(name),
                    gyro_bias_dps,
                    samples: summary.samples,
                };
                report.stored = storage.store(&report.id.key(), &cal).await;
                report.bias_dps = gyro_bias_dps;
            }
        }
        reports
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

/// Sensor-to-board axis remap for the LSM6DSO32 on this board: negate x and z.
fn sensor_to_board([x, y, z]: [f32; 3]) -> [f32; 3] {
    [-x, y, -z]
}
