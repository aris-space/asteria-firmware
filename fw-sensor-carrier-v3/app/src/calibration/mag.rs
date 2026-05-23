use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::Duration;
use firmware_params::{Param, make_key};
use nalgebra::{Matrix3, Vector3};
use postcard::experimental::max_size::MaxSize;
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

use crate::sensors::{MAG_BUS_1, MAG_BUS_2, MagnetometerId};
use crate::types::{MagSample, RawMagSample};

type Mutex = CriticalSectionRawMutex;

/// On-wire representation of a magnetometer's iron correction. Held in a
/// `Param` so it can be tuned and persisted; the soft-iron matrix also
/// absorbs any residual mounting rotation, so we don't carry a separate
/// fine-rot here.
#[derive(Clone, Copy, Serialize, Deserialize, MaxSize, Schema)]
pub struct MagCalWire {
    pub hard_iron: [f32; 3],
    pub soft_iron: [f32; 9],
}

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

#[allow(clippy::excessive_precision)]
const MAG_0_DEFAULT: MagCalWire = MagCalWire {
    hard_iron: [
        -5548.220872821729_f32,
        3827.5558491680104_f32,
        1103.1746334467373_f32,
    ],
    soft_iron: [
        0.9933533874801035_f32,
        0.02660683424766733_f32,
        -0.012040160633120614_f32,
        0.0266068342476673_f32,
        1.0749237129791611_f32,
        -0.004425871227641531_f32,
        -0.01204016063312053_f32,
        -0.0044258712276415224_f32,
        0.9896712863098899_f32,
    ],
};

#[allow(clippy::excessive_precision)]
const MAG_1_DEFAULT: MagCalWire = MagCalWire {
    hard_iron: [
        1877.0587380839947_f32,
        7553.287522291296_f32,
        6455.773615317315_f32,
    ],
    soft_iron: [
        0.9947437765775108_f32,
        0.02142849359925576_f32,
        0.013753388737255394_f32,
        0.021428493599255673_f32,
        1.056768106384783_f32,
        -0.003894936674428707_f32,
        0.013753388737255299_f32,
        -0.003894936674428721_f32,
        0.999343144534608_f32,
    ],
};

pub static MAG_0_PARAM: Param<Mutex, MagCalWire> = Param::new(make_key("v1/mag/0/cal"));
pub static MAG_1_PARAM: Param<Mutex, MagCalWire> = Param::new(make_key("v1/mag/1/cal"));

fn load(id: MagnetometerId) -> MagCalWire {
    match id {
        MAG_BUS_1 => MAG_0_PARAM.get_or(MAG_0_DEFAULT),
        MAG_BUS_2 => MAG_1_PARAM.get_or(MAG_1_DEFAULT),
        _ => unreachable!(),
    }
}

/// Turn a raw mag sample (nT in sensor frame) into a calibrated,
/// board-frame `MagSample` (nT). Sensor-to-board axis remap on this board
/// is a negation of all three axes; the soft-iron matrix then carries
/// both the iron correction and any residual mounting rotation.
pub fn apply_calibration(raw: RawMagSample) -> MagSample {
    let cal = load(raw.src);
    let hard_iron = Vector3::from(cal.hard_iron);
    let soft_iron = Matrix3::from_row_slice(&cal.soft_iron);
    let board = -Vector3::new(raw.x, raw.y, raw.z);
    let corrected = soft_iron * (board - hard_iron);
    MagSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        x: corrected.x,
        y: corrected.y,
        z: corrected.z,
    }
}
