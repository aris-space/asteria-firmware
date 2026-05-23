use embassy_time::Duration;

use crate::sensors::{DHT_0, DHT_1, DhtId};
use crate::types::{DhtSample, RawDhtSample};

#[derive(Clone, Copy)]
struct DhtCal {
    /// How long ago (relative to read-completion time) the physical
    /// measurement actually happened.
    delay: Duration,
}

const DHT_0_CAL: DhtCal = DhtCal {
    delay: Duration::from_millis(0),
};
const DHT_1_CAL: DhtCal = DhtCal {
    delay: Duration::from_millis(0),
};

fn load(id: DhtId) -> DhtCal {
    match id {
        DHT_0 => DHT_0_CAL,
        DHT_1 => DHT_1_CAL,
        _ => unreachable!(),
    }
}

pub fn apply_calibration(raw: RawDhtSample) -> DhtSample {
    let cal = load(raw.src);
    DhtSample {
        src: raw.src,
        ts: raw.ts - cal.delay,
        temperature_c: raw.temperature_c,
        humidity_rh: raw.humidity_rh,
    }
}
