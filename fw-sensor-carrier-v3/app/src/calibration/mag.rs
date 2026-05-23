use embassy_time::Duration;
use nalgebra::{Matrix3, Vector3};

use crate::sensors::{MAGNETOMETER_0, MAGNETOMETER_1, MagnetometerId};
use crate::types::{MagSample, RawMagSample};

#[derive(Clone, Copy)]
struct MagCal {
    /// How long ago (relative to read-completion time) the physical
    /// measurement actually happened.
    delay: Duration,
    hard_iron: Vector3<f32>,
    soft_iron: Matrix3<f32>,
}

#[allow(clippy::excessive_precision)]
const MAG_0_CAL: MagCal = MagCal {
    delay: Duration::from_millis(0), // TODO: calibrate
    hard_iron: Vector3::new(
        -5548.220872821729_f32,
        3827.5558491680104_f32,
        1103.1746334467373_f32,
    ),
    soft_iron: Matrix3::new(
        0.9933533874801035_f32,
        0.02660683424766733_f32,
        -0.012040160633120614_f32,
        0.0266068342476673_f32,
        1.0749237129791611_f32,
        -0.004425871227641531_f32,
        -0.01204016063312053_f32,
        -0.0044258712276415224_f32,
        0.9896712863098899_f32,
    ),
};

#[allow(clippy::excessive_precision)]
const MAG_1_CAL: MagCal = MagCal {
    delay: Duration::from_millis(0), // TODO: calibrate
    hard_iron: Vector3::new(
        1877.0587380839947_f32,
        7553.287522291296_f32,
        6455.773615317315_f32,
    ),
    soft_iron: Matrix3::new(
        0.9947437765775108_f32,
        0.02142849359925576_f32,
        0.013753388737255394_f32,
        0.021428493599255673_f32,
        1.056768106384783_f32,
        -0.003894936674428707_f32,
        0.013753388737255299_f32,
        -0.003894936674428721_f32,
        0.999343144534608_f32,
    ),
};

fn load(id: MagnetometerId) -> MagCal {
    match id {
        MAGNETOMETER_0 => MAG_0_CAL,
        MAGNETOMETER_1 => MAG_1_CAL,
        _ => unreachable!(),
    }
}

/// Turn a raw mag sample (nT in sensor frame) into a calibrated,
/// board-frame `MagSample` (nT). `raw.ts` is when the readout finished
/// the I/O; the cal-owned delay is subtracted to recover the physical
/// measurement time. Sensor-to-board axis remap on this board is a
/// negation of all three axes.
pub fn apply_calibration(raw: RawMagSample) -> MagSample {
    let cal = load(raw.src);
    let board = -Vector3::new(raw.x, raw.y, raw.z);
    let corrected = cal.soft_iron * (board - cal.hard_iron);
    MagSample {
        src: raw.src,
        ts: raw.ts - cal.delay,
        x: corrected.x,
        y: corrected.y,
        z: corrected.z,
    }
}
