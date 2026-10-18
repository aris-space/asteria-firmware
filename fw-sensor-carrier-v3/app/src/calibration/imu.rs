//! IMU calibration: latency, LSM6DSO32 units, axes, gyro bias, and
//! accelerometer offset and scale.

use core::fmt;

use lsm6dso32::{Acceleration, AngularRate};
use serde::{Deserialize, Serialize};

use super::{Calibrations, Floats, parse_floats};
use crate::sensors::{IMU_COUNT, ImuId};
use crate::tasks::readout::imu::{ACCEL_FULL_SCALE, GYRO_FULL_SCALE};
use crate::types::{ImuSample, RawImuSample};

// Plausible accelerometer correction, as in calibrate.py; a pasted line
// outside these bounds is rejected rather than applied.
const MAX_ACCEL_OFFSET_G: f32 = 0.2;
const ACCEL_SCALE_RANGE: core::ops::RangeInclusive<f32> = 0.9..=1.1;

/// Sensor-to-board axis remap for the LSM6DSO32 on this board: negate x and z.
fn sensor_to_board([x, y, z]: [f32; 3]) -> [f32; 3] {
    [-x, y, -z]
}

/// Gyro offsets in degrees per second, and the accelerometer correction applied
/// per axis as `(board - accel_offset_g) * accel_scale`, all in board axes.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Correction {
    gyro_bias_dps: [f32; 3],
    accel_offset_g: [f32; 3],
    accel_scale: [f32; 3],
}

impl super::Correction for Correction {
    const DEFAULT: Self = Self {
        gyro_bias_dps: [0.0; 3],
        accel_offset_g: [0.0; 3],
        accel_scale: [1.0; 3],
    };
    const FIELDS: &'static [&'static str] = &["gyro_bias_dps", "accel_offset_g", "accel_scale"];

    fn set(&mut self, key: &str, value: &str) -> bool {
        let field = match key {
            "gyro_bias_dps" => &mut self.gyro_bias_dps,
            "accel_offset_g" => &mut self.accel_offset_g,
            "accel_scale" => &mut self.accel_scale,
            _ => return false,
        };
        parse_floats(value).map(|v| *field = v).is_some()
    }

    fn is_valid(&self) -> bool {
        self.gyro_bias_dps.iter().all(|v| v.is_finite())
            && self
                .accel_offset_g
                .iter()
                .all(|v| v.abs() <= MAX_ACCEL_OFFSET_G)
            && self
                .accel_scale
                .iter()
                .all(|v| ACCEL_SCALE_RANGE.contains(v))
    }
}

impl fmt::Display for Correction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            " gyro_bias_dps={} accel_offset_g={} accel_scale={}",
            Floats(&self.gyro_bias_dps),
            Floats(&self.accel_offset_g),
            Floats(&self.accel_scale)
        )
    }
}

pub static CAL: Calibrations<ImuId, Correction, IMU_COUNT> = Calibrations::new(ImuId::ALL);

pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    let cal = CAL.applied(raw.src);
    let accel = Acceleration::from_raw(raw.accel, ACCEL_FULL_SCALE);
    let gyro = AngularRate::from_raw(raw.gyro, GYRO_FULL_SCALE);
    let correction = cal.correction;
    let board = sensor_to_board([accel.x, accel.y, accel.z]);
    let [ax, ay, az] = core::array::from_fn(|axis| {
        (board[axis] - correction.accel_offset_g[axis]) * correction.accel_scale[axis]
    });
    let [gx, gy, gz] = sensor_to_board([gyro.x, gyro.y, gyro.z]);
    let bias = correction.gyro_bias_dps;
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
