use embassy_time::Duration;

use crate::types::{BaroSample, RawBaroSample};

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(20); // TODO: calibrate

pub fn apply_calibration(raw: RawBaroSample) -> BaroSample {
    BaroSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        pressure_mbar: raw.pressure_mbar,
        temperature_c: raw.temperature_c,
    }
}
