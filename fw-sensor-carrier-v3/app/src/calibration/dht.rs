use crate::types::{DhtSample, RawDhtSample};

// Measurement latency has not been measured, so retain the read-completion
// timestamp rather than applying an assumed offset.
pub fn apply_calibration(raw: RawDhtSample) -> DhtSample {
    DhtSample {
        src: raw.src,
        ts: raw.ts,
        temperature_c: raw.temperature_c,
        humidity_rh: raw.humidity_rh,
    }
}
