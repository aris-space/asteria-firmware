use embassy_time::Duration;

use crate::types::{GnssSample, RawGnssSample};

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
// The receiver's actual output latency has not been measured. Use reception
// time rather than backdating every fix by an assumed 100 ms.
const DELAY: Duration = Duration::from_millis(0);

pub fn apply_calibration(raw: RawGnssSample) -> GnssSample {
    GnssSample {
        src: raw.src,
        // Keep checked subtraction for a future measured latency.
        ts: raw.ts.checked_sub(DELAY).unwrap_or(raw.ts),
        pvt: raw.pvt,
    }
}
