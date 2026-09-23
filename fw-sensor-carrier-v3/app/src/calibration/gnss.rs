use embassy_time::Duration;

use crate::types::{GnssSample, RawGnssSample};

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(100); // TODO: calibrate

pub fn apply_calibration(raw: RawGnssSample) -> GnssSample {
    GnssSample {
        src: raw.src,
        // A receiver can report its first fix before the 100 ms delay has
        // elapsed since boot. Use read time for that startup sample.
        ts: raw.ts.checked_sub(DELAY).unwrap_or(raw.ts),
        pvt: raw.pvt,
    }
}
