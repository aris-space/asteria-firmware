use defmt::trace;
use embassy_futures::select::{Either, select};

use super::filters::MovingAverage;
use crate::measurements::PressureSample;
use crate::sensors::{BAROMETER_0, BAROMETER_1};
use crate::signals;

const BAROMETER_SAMPLE_HZ: usize = 40;
const MOVING_AVERAGE_COUNT: usize = (BAROMETER_SAMPLE_HZ + 4) / 5; // ~5 Hz cutoff

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut sub0 = signals::PRESSURE_CHANNELS[BAROMETER_0.index()]
        .subscriber()
        .expect("pressure: failed to subscribe to barometer 0");
    let mut sub1 = signals::PRESSURE_CHANNELS[BAROMETER_1.index()]
        .subscriber()
        .expect("pressure: failed to subscribe to barometer 1");

    let mut filter = MovingAverage::<f32, MOVING_AVERAGE_COUNT>::new();
    let sender = signals::PRESSURE_FUSED_WATCH.sender();

    loop {
        let sample: PressureSample =
            match select(sub0.next_message_pure(), sub1.next_message_pure()).await {
                Either::First(s) | Either::Second(s) => s,
            };

        let filtered = filter.update(sample.data.value.pressure_mbar);
        sender.send(filtered);
        trace!("pressure: filtered={} mbar", filtered);
    }
}
