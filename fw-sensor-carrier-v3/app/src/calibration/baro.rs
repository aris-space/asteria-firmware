use embassy_time::Duration;

use crate::sensors::{BAROMETER_0, BAROMETER_1, BarometerId};
use crate::types::{BaroSample, RawBaroSample};

#[derive(Clone, Copy)]
struct BaroCal {
    /// How long ago (relative to read-completion time) the physical
    /// measurement actually happened.
    delay: Duration,
}

const BARO_0_CAL: BaroCal = BaroCal {
    delay: Duration::from_millis(20), // TODO: calibrate
};
const BARO_1_CAL: BaroCal = BaroCal {
    delay: Duration::from_millis(20), // TODO: calibrate
};

fn load(id: BarometerId) -> BaroCal {
    match id {
        BAROMETER_0 => BARO_0_CAL,
        BAROMETER_1 => BARO_1_CAL,
        _ => unreachable!(),
    }
}

pub fn apply_calibration(raw: RawBaroSample) -> BaroSample {
    let cal = load(raw.src);
    BaroSample {
        src: raw.src,
        ts: raw.ts - cal.delay,
        pressure_mbar: raw.pressure_mbar,
        temperature_c: raw.temperature_c,
    }
}
