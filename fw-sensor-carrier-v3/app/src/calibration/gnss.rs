//! GNSS calibration: latency only.

use core::fmt;

use serde::{Deserialize, Serialize};

use super::Calibrations;
use crate::sensors::{GNSS_COUNT, GnssId};
use crate::types::{GnssSample, RawGnssSample};

#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Correction;

impl super::Correction for Correction {
    const DEFAULT: Self = Self;
}

impl fmt::Display for Correction {
    fn fmt(&self, _: &mut fmt::Formatter<'_>) -> fmt::Result {
        Ok(())
    }
}

pub static CAL: Calibrations<GnssId, Correction, GNSS_COUNT> = Calibrations::new(GnssId::ALL);

pub fn apply_calibration(raw: RawGnssSample) -> GnssSample {
    let cal = CAL.applied(raw.src);
    GnssSample {
        src: raw.src,
        ts: cal.sample_time(raw.ts),
        pvt: raw.pvt,
    }
}
