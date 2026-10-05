//! Humidity sensor calibration: latency only.

use core::fmt;

use serde::{Deserialize, Serialize};

use super::Calibrations;
use crate::sensors::{DHT_COUNT, DhtId};
use crate::types::{DhtSample, RawDhtSample};

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

pub static CAL: Calibrations<DhtId, Correction, DHT_COUNT> = Calibrations::new(DhtId::ALL);

pub fn apply_calibration(raw: RawDhtSample) -> DhtSample {
    let cal = CAL.applied(raw.src);
    DhtSample {
        src: raw.src,
        ts: cal.sample_time(raw.ts),
        temperature_c: raw.temperature_c,
        humidity_rh: raw.humidity_rh,
    }
}
