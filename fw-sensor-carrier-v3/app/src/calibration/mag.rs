//! Magnetometer calibration: latency, LSM303AGR units, axes, hard- and
//! soft-iron correction, and the local field strength.

use core::fmt;

use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use super::{Calibrations, Floats, parse_floats};
use crate::sensors::{MAG_COUNT, MagId};
use crate::types::{MagSample, RawMagSample};

// The LSM303AGR reports 150 nT per count.
const LSB_TO_NT: f32 = 150.0;
// Samples whose field strength differs from the calibrated one by more than
// this fraction are disturbed (nearby iron or magnets) and not fused.
const MAX_FIELD_ERROR: f32 = 0.1;

/// Sensor-to-board axis remap on this board: negate all three axes.
fn sensor_to_board(counts: [i16; 3]) -> [i16; 3] {
    counts.map(i16::saturating_neg)
}

/// Hard- and soft-iron correction, applied as `soft_iron * (board - hard_iron)`
/// in nT. The soft-iron matrix also absorbs any residual mounting rotation.
/// `field_nt` is the field strength where the calibration was made.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Correction {
    hard_iron_nt: [f32; 3],
    soft_iron: [f32; 9],
    field_nt: f32,
}

impl super::Correction for Correction {
    const DEFAULT: Self = Self {
        hard_iron_nt: [0.0; 3],
        soft_iron: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        field_nt: 0.0,
    };
    const FIELDS: &'static [&'static str] = &["hard_nt", "soft", "field_nt"];

    fn set(&mut self, key: &str, value: &str) -> bool {
        match key {
            "hard_nt" => parse_floats(value).map(|v| self.hard_iron_nt = v).is_some(),
            "soft" => parse_floats(value).map(|v| self.soft_iron = v).is_some(),
            "field_nt" => parse_floats(value).map(|[v]| self.field_nt = v).is_some(),
            _ => false,
        }
    }

    fn is_valid(&self) -> bool {
        self.hard_iron_nt
            .iter()
            .chain(&self.soft_iron)
            .all(|v| v.is_finite())
            && self.field_nt.is_finite()
            && self.field_nt >= 0.0
    }
}

impl Correction {
    /// Whether this is a measured correction rather than the identity default.
    pub fn is_calibrated(&self) -> bool {
        *self != <Self as super::Correction>::DEFAULT
    }

    /// Accept only calibrated samples whose field strength matches the
    /// calibrated one.
    pub fn accepts_field(&self, field_nt: f32) -> bool {
        self.is_calibrated()
            && libm::fabsf(field_nt - self.field_nt) <= MAX_FIELD_ERROR * self.field_nt
    }

    fn correct_board_field(&self, board: Vector3<f32>) -> Vector3<f32> {
        let hard_iron = Vector3::from(self.hard_iron_nt);
        let soft_iron = Matrix3::from_row_slice(&self.soft_iron);
        soft_iron * (board - hard_iron)
    }
}

impl fmt::Display for Correction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            " hard_nt={} soft={} field_nt={}",
            Floats(&self.hard_iron_nt),
            Floats(&self.soft_iron),
            self.field_nt
        )
    }
}

pub static CAL: Calibrations<MagId, Correction, MAG_COUNT> = Calibrations::new(MagId::ALL);

pub fn apply_calibration(raw: RawMagSample) -> MagSample {
    let cal = CAL.applied(raw.src);
    let board = sensor_to_board([raw.x, raw.y, raw.z]).map(|c| c as f32 * LSB_TO_NT);
    let corrected = cal.correction.correct_board_field(Vector3::from(board));
    MagSample {
        src: raw.src,
        ts: cal.sample_time(raw.ts),
        x: corrected.x,
        y: corrected.y,
        z: corrected.z,
    }
}
