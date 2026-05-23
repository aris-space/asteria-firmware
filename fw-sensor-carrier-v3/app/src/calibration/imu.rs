use embassy_time::Duration;
use nalgebra::{Matrix3, Vector3};

use crate::sensors::{IMU_0, IMU_1, ImuId};
use crate::types::{ImuSample, RawImuSample};

#[derive(Clone, Copy)]
struct ImuCal {
    /// How long ago (relative to read-completion time) the physical
    /// measurement actually happened.
    delay: Duration,
    /// Residual rotation from the post-axis-flip sensor frame to the
    /// ideal board frame. Identity until populated by per-board cal.
    fine_rot: Matrix3<f32>,
}

const IMU_0_CAL: ImuCal = ImuCal {
    delay: Duration::from_millis(0),
    fine_rot: Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
};
const IMU_1_CAL: ImuCal = ImuCal {
    delay: Duration::from_millis(0),
    fine_rot: Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
};

fn load(id: ImuId) -> ImuCal {
    match id {
        IMU_0 => IMU_0_CAL,
        IMU_1 => IMU_1_CAL,
        _ => unreachable!(),
    }
}

/// Sensor-to-board coarse axis remap for the LSM6DSO32 on this board:
/// negate x and z, keep y.
fn sensor_to_board(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(-v.x, v.y, -v.z)
}

pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    let cal = load(raw.src);
    let accel = cal.fine_rot * sensor_to_board(Vector3::new(raw.accel.x, raw.accel.y, raw.accel.z));
    let gyro = cal.fine_rot * sensor_to_board(Vector3::new(raw.gyro.x, raw.gyro.y, raw.gyro.z));
    ImuSample {
        src: raw.src,
        ts: raw.ts - cal.delay,
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
