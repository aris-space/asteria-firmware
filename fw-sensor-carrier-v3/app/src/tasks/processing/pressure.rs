use defmt::trace;
use embassy_futures::select::{Either, select};

use super::filters::MovingAverage;
use crate::measurements::{Pressure, PressureSample};
use crate::sensors::{BAROMETER_0, BAROMETER_1};
use crate::signals;
use crate::tasks::readout::barometer::SAMPLE_HZ as BAROMETER_HZ;

/// Window length for a ~5 Hz output cutoff at the barometer's sample rate.
const MOVING_AVERAGE_COUNT: usize = (BAROMETER_HZ as usize + 4) / 5;

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut sub0 = signals::PRESSURE_CHANNELS[BAROMETER_0.index()]
        .subscriber()
        .expect("pressure: failed to subscribe to barometer 0");
    let mut sub1 = signals::PRESSURE_CHANNELS[BAROMETER_1.index()]
        .subscriber()
        .expect("pressure: failed to subscribe to barometer 1");

    let mut filter = MovingAverage::<f32, MOVING_AVERAGE_COUNT>::new();
    let sender = signals::PRESSURE_WATCH.sender();

    loop {
        let sample: PressureSample =
            match select(sub0.next_message_pure(), sub1.next_message_pure()).await {
                Either::First(s) | Either::Second(s) => s,
            };

        let filtered = filter.update(sample.pressure_mbar);
        let out = Pressure {
            ts: sample.ts,
            mbar: filtered,
        };
        sender.send(out);
        trace!("pressure: filtered={} mbar", filtered);
    }
}
