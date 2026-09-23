use crate::types::{GnssSample, RawGnssSample};

// The receiver's actual output latency has not been measured. Use reception
// time rather than backdating every fix by an assumed 100 ms.
pub fn apply_calibration(raw: RawGnssSample) -> GnssSample {
    GnssSample {
        src: raw.src,
        ts: raw.ts,
        pvt: raw.pvt,
    }
}
