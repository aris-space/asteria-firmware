use embassy_time::Duration;

use crate::sensors::{GNSS_0, GNSS_1, GnssId};
use crate::types::{GnssSample, RawGnssSample};

#[derive(Clone, Copy)]
struct GnssCal {
    /// How long ago (relative to read-completion time) the physical
    /// measurement actually happened.
    delay: Duration,
}

const GNSS_0_CAL: GnssCal = GnssCal {
    delay: Duration::from_millis(100), // TODO: calibrate
};
const GNSS_1_CAL: GnssCal = GnssCal {
    delay: Duration::from_millis(100), // TODO: calibrate
};

fn load(id: GnssId) -> GnssCal {
    match id {
        GNSS_0 => GNSS_0_CAL,
        GNSS_1 => GNSS_1_CAL,
        _ => unreachable!(),
    }
}

pub fn apply_calibration(raw: RawGnssSample) -> GnssSample {
    let cal = load(raw.src);
    GnssSample {
        src: raw.src,
        ts: raw.ts - cal.delay,
        pvt: raw.pvt,
    }
}
