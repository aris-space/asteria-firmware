use embassy_time::Duration;
use lsm6dso32::{Acceleration, AngularRate};

use crate::types::{ImuSample, RawImuSample};

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    ImuSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        // Sensor-to-board axis remap for the LSM6DSO32 on this board: negate x and z.
        accel: Acceleration {
            x: -raw.accel.x,
            y: raw.accel.y,
            z: -raw.accel.z,
        },
        gyro: AngularRate {
            x: -raw.gyro.x,
            y: raw.gyro.y,
            z: -raw.gyro.z,
        },
    }
}
