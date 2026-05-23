use embassy_time::Duration;

use crate::types::{DhtSample, RawDhtSample};

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

pub fn apply_calibration(raw: RawDhtSample) -> DhtSample {
    DhtSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        temperature_c: raw.temperature_c,
        humidity_rh: raw.humidity_rh,
    }
}
