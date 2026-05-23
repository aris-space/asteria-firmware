use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Duration;
use firmware_params::{Param, make_key};
use nalgebra::{Matrix3, Vector3};
use postcard::experimental::max_size::MaxSize;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

use crate::sensors::{IMU_0, IMU_1, ImuId};
use crate::types::{ImuSample, RawImuSample};

type Mutex = CriticalSectionRawMutex;

/// On-wire representation of an IMU's residual rotation from the
/// post-axis-flip sensor frame to the ideal board frame. Identity until
/// populated by per-board cal.
#[derive(Clone, Copy, Serialize, Deserialize, MaxSize, Schema)]
pub struct ImuCalWire {
    pub fine_rot: [f32; 9],
}

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

const IDENTITY: ImuCalWire = ImuCalWire {
    fine_rot: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
};

pub static IMU_0_PARAM: Param<Mutex, ImuCalWire> = Param::new(make_key("v1/imu/0/cal"));
pub static IMU_1_PARAM: Param<Mutex, ImuCalWire> = Param::new(make_key("v1/imu/1/cal"));

fn load(id: ImuId) -> ImuCalWire {
    match id {
        IMU_0 => IMU_0_PARAM.get_or(IDENTITY),
        IMU_1 => IMU_1_PARAM.get_or(IDENTITY),
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
