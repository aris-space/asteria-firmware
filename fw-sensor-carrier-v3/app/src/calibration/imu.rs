//! IMU calibration: latency, LSM6DSO32 units, axes, and gyro bias.

use core::fmt;

use lsm6dso32::{Acceleration, AngularRate};
use serde::{Deserialize, Serialize};

use super::{Calibrations, Floats, parse_floats};
use crate::sensors::{IMU_COUNT, ImuId};
use crate::tasks::readout::imu::{ACCEL_FULL_SCALE, GYRO_FULL_SCALE};
use crate::types::{ImuSample, RawImuSample};

/// Sensor-to-board axis remap for the LSM6DSO32 on this board: negate x and z.
fn sensor_to_board([x, y, z]: [f32; 3]) -> [f32; 3] {
    [-x, y, -z]
}

/// Gyro offsets in board axes and degrees per second.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Correction {
    gyro_bias_dps: [f32; 3],
}

impl super::Correction for Correction {
    const DEFAULT: Self = Self {
        gyro_bias_dps: [0.0; 3],
    };
    const FIELDS: &'static [&'static str] = &["gyro_bias_dps"];

    fn set(&mut self, key: &str, value: &str) -> bool {
        match (key, parse_floats(value)) {
            ("gyro_bias_dps", Some(bias)) => self.gyro_bias_dps = bias,
            _ => return false,
        }
        true
    }

    fn is_valid(&self) -> bool {
        self.gyro_bias_dps.iter().all(|v| v.is_finite())
    }
}

impl fmt::Display for Correction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, " gyro_bias_dps={}", Floats(&self.gyro_bias_dps))
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
