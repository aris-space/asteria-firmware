use embassy_time::Duration;

use crate::sensors::{IMU_0, IMU_1, ImuId};
use crate::types::{ImuSample, RawImuSample};

#[derive(Clone, Copy)]
struct ImuCal {
    /// How long ago (relative to read-completion time) the physical
    /// measurement actually happened.
    delay: Duration,
}

const IMU_0_CAL: ImuCal = ImuCal {
    delay: Duration::from_millis(0),
};
const IMU_1_CAL: ImuCal = ImuCal {
    delay: Duration::from_millis(0),
};

fn load(id: ImuId) -> ImuCal {
    match id {
        IMU_0 => IMU_0_CAL,
        IMU_1 => IMU_1_CAL,
        _ => unreachable!(),
    }
}

pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    let cal = load(raw.src);
    // Sensor -> board frame: flip x and z on both accel and gyro.
    ImuSample {
        src: raw.src,
        ts: raw.ts - cal.delay,
        accel: lsm6dso32::Acceleration {
            x: -raw.accel.x,
            y: raw.accel.y,
            z: -raw.accel.z,
        },
        gyro: lsm6dso32::AngularRate {
            x: -raw.gyro.x,
            y: raw.gyro.y,
            z: -raw.gyro.z,
        },
    }
}
