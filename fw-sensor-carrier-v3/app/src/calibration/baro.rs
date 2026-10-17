//! Barometer calibration: latency only. The MS5607 compensates its own output.

use core::fmt;

use serde::{Deserialize, Serialize};

use super::Calibrations;
use crate::sensors::{BARO_COUNT, BaroId};
use crate::types::{BaroSample, RawBaroSample};

#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Correction;

impl super::Correction for Correction {
    const DEFAULT: Self = Self;
    const FIELDS: &'static [&'static str] = &[];

    fn set(&mut self, _: &str, _: &str) -> bool {
        false
    }
}

impl fmt::Display for Correction {
    fn fmt(&self, _: &mut fmt::Formatter<'_>) -> fmt::Result {
        Ok(())
    }
}

pub static CAL: Calibrations<BaroId, Correction, BARO_COUNT> = Calibrations::new(BaroId::ALL);

pub fn apply_calibration(raw: RawBaroSample) -> BaroSample {
    let cal = CAL.applied(raw.src);
    BaroSample {
        src: raw.src,
        ts: cal.sample_time(raw.ts),
        pressure_mbar: raw.pressure_mbar,
        temperature_c: raw.temperature_c,
    }
}
