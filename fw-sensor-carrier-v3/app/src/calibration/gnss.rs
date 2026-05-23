use embassy_time::Duration;

use crate::types::{GnssSample, RawGnssSample};

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(100); // TODO: calibrate

pub fn apply_calibration(raw: RawGnssSample) -> GnssSample {
    GnssSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        pvt: raw.pvt,
    }
}
