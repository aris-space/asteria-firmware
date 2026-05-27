use embassy_time::Duration;
use lsm6dso32::{Acceleration, AngularRate};

use crate::types::{ImuSample, RawImuSample};

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    let [ax, ay, az] = sensor_to_board([raw.accel.x, raw.accel.y, raw.accel.z]);
    let [gx, gy, gz] = sensor_to_board([raw.gyro.x, raw.gyro.y, raw.gyro.z]);
    ImuSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        accel: Acceleration {
            x: ax,
            y: ay,
            z: az,
        },
        gyro: AngularRate {
            x: gx,
            y: gy,
            z: gz,
        },
    }
}

/// Sensor-to-board axis remap for the LSM6DSO32 on this board: negate x and z.
fn sensor_to_board([x, y, z]: [f32; 3]) -> [f32; 3] {
    [-x, y, -z]
}
