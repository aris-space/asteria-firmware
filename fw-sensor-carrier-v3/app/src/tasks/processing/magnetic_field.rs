use defmt::trace;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant};

use crate::sensors::{MAGNETOMETER_0, MAGNETOMETER_1, MagnetometerId};
use crate::signals;
use crate::types::MagSample;

/// Keep using the current primary; only switch on timeout.
const MAG_TIMEOUT: Duration = Duration::from_millis(100);

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut sub0 = signals::MAG_CHANNELS[MAGNETOMETER_0.index()]
        .subscriber()
        .expect("too many subs on MAG_CHANNELS; increase SUBS");
    let mut sub1 = signals::MAG_CHANNELS[MAGNETOMETER_1.index()]
        .subscriber()
        .expect("too many subs on MAG_CHANNELS; increase SUBS");

    let mut selector = TimeoutSelector::new(MAG_TIMEOUT);
    let sender = signals::MAG_WATCH.sender();

    loop {
        let sample: MagSample =
            match select(sub0.next_message_pure(), sub1.next_message_pure()).await {
                Either::First(s) | Either::Second(s) => s,
            };

        if !selector.accept(sample.src, sample.ts) {
            continue;
        }

        sender.send(sample);
        trace!("mag: nt x={} y={} z={}", sample.x, sample.y, sample.z);
    }
}

struct TimeoutSelector {
    primary: MagnetometerId,
    last_update: Instant,
    timeout: Duration,
}

impl TimeoutSelector {
    fn new(timeout: Duration) -> Self {
        Self {
            primary: MAGNETOMETER_0,
            last_update: Instant::now(),
            timeout,
        }
    }

    fn accept(&mut self, id: MagnetometerId, ts: Instant) -> bool {
        if self.primary == id || self.last_update.elapsed() > self.timeout {
            self.primary = id;
            self.last_update = ts;
            true
        } else {
            false
        }
    }
}
