//! Picks one sensor of each redundant pair (barometer, magnetometer, DHT) and
//! publishes its readings to the `SELECTED_*` watches in [`signals`].

use asteria_sef_core::{Selector, SelectorConfig};
use embassy_futures::select::{Either3, select3};
use embassy_sync::pubsub::WaitResult;
use embassy_time::Duration;

use crate::signals;

// A sensor silent for this long hands over to the other one of its pair;
// a few of its sample intervals. Both sensors report the same score, so
// nothing else hands over.
const BARO_TIMEOUT: Duration = Duration::from_millis(100);
const MAG_TIMEOUT: Duration = Duration::from_millis(300);
const DHT_TIMEOUT: Duration = Duration::from_millis(2_500);

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut baro = signals::BARO_CHANNEL
        .subscriber()
        .expect("selection: barometer subscriber slot");
    let mut mag = signals::MAG_CHANNEL
        .subscriber()
        .expect("selection: magnetometer subscriber slot");
    let mut dht = signals::DHT_CHANNEL
        .subscriber()
        .expect("selection: DHT subscriber slot");
    let mut baro_selector = Selector::<2>::new(SelectorConfig {
        maximum_age_us: BARO_TIMEOUT.as_micros(),
        ..SelectorConfig::default()
    });
    let mut mag_selector = Selector::<2>::new(SelectorConfig {
        maximum_age_us: MAG_TIMEOUT.as_micros(),
        ..SelectorConfig::default()
    });
    let mut dht_selector = Selector::<2>::new(SelectorConfig {
        maximum_age_us: DHT_TIMEOUT.as_micros(),
        ..SelectorConfig::default()
    });
    loop {
        match select3(baro.next_message(), mag.next_message(), dht.next_message()).await {
            Either3::First(WaitResult::Message(reading)) => {
                let raw = &reading.raw;
                if baro_selector.report(raw.src.index(), raw.read_ts.as_micros(), 0.0) {
                    signals::SELECTED_BARO.sender().send(reading);
                }
            }
            Either3::Second(WaitResult::Message(reading)) => {
                let raw = &reading.raw;
                if mag_selector.report(raw.src.index(), raw.read_ts.as_micros(), 0.0) {
                    signals::SELECTED_MAG.sender().send(reading);
                }
            }
            Either3::Third(WaitResult::Message(reading)) => {
                let raw = &reading.raw;
                if dht_selector.report(raw.src.index(), raw.read_ts.as_micros(), 0.0) {
                    signals::SELECTED_DHT.sender().send(reading);
                }
            }
            // Lost readings: the next one is as good.
            _ => {}
        }
    }
}
