use defmt::{Debug2Format, info};
use embassy_sync::once_lock::OnceLock;
use embassy_time::Duration;
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use crate::sensors::IMU_COUNT;
use crate::storage::{self, Storage};
use crate::types::{ImuSample, RawImuSample};

/// Residual rotation from the post-axis-flip sensor frame to the ideal
/// board frame. Identity until populated by per-board cal.
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuCalWire {
    pub fine_rot: [f32; 9],
}

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

const IDENTITY: ImuCalWire = ImuCalWire {
    fine_rot: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
};

const DEFAULTS: [ImuCalWire; IMU_COUNT] = [IDENTITY, IDENTITY];
const KEYS: [storage::Key; IMU_COUNT] = [storage::key("imu0"), storage::key("imu1")];

/// Live per-IMU cal, written once at startup. A fresh cal persists to flash
/// but does not touch this; a reset reloads and applies it.
static CAL: OnceLock<[ImuCalWire; IMU_COUNT]> = OnceLock::new();

/// Read each IMU's stored cal (or identity) and publish it for the readout to
/// apply. Call once at startup, before the readout tasks run.
pub async fn load(storage: &Storage) {
    let cal = [
        load_one(storage, &KEYS[0], 0).await,
        load_one(storage, &KEYS[1], 1).await,
    ];
    let _ = CAL.init(cal);
}

/// Load one IMU's cal, logging whether it came from flash or fell back to the
/// identity (no-correction) default.
async fn load_one(storage: &Storage, key: &storage::Key, idx: usize) -> ImuCalWire {
    match storage.load::<ImuCalWire>(key).await {
        Some(cal) => {
            info!(
                "IMU {}: fine-rot cal loaded from flash: {}",
                idx,
                Debug2Format(&cal.fine_rot)
            );
            cal
        }
        None => {
            info!(
                "IMU {}: no cal in flash, using identity (no correction)",
                idx
            );
            IDENTITY
        }
    }
}

/// Sensor-to-board coarse axis remap for the LSM6DSO32 on this board:
/// negate x and z, keep y.
fn sensor_to_board(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(-v.x, v.y, -v.z)
}

pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    let cal = CAL.try_get().unwrap_or(&DEFAULTS)[raw.src.index()];
    let fine_rot = Matrix3::from_row_slice(&cal.fine_rot);
    let accel = fine_rot * sensor_to_board(Vector3::new(raw.accel.x, raw.accel.y, raw.accel.z));
    let gyro = fine_rot * sensor_to_board(Vector3::new(raw.gyro.x, raw.gyro.y, raw.gyro.z));
    ImuSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        accel: lsm6dso32::Acceleration {
            x: accel.x,
            y: accel.y,
            z: accel.z,
        },
        gyro: lsm6dso32::AngularRate {
            x: gyro.x,
            y: gyro.y,
            z: gyro.z,
        },
    }
}
