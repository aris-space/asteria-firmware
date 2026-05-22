use defmt::trace;
use embassy_futures::select::{Either, select};
use embassy_time::Instant;

use crate::filters::Ema;
use crate::sensors::{BAROMETER_0, BAROMETER_1};
use crate::signals;
use crate::tasks::readout::barometer::SAMPLE_HZ as BAROMETER_HZ;
use crate::types::{BaroSample, Pressure};

/// EMA time constant. Sets a sample-rate-independent low-pass with
/// 3 dB cutoff ~ 1 / (2 * pi * tau) Hz.
const PRESSURE_TAU_S: f32 = 1.0 / (2.0 * core::f32::consts::PI * 5.0);

/// First-sample fallback dt; matches the nominal barometer rate.
const FALLBACK_DT_S: f32 = 1.0 / BAROMETER_HZ as f32;

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut sub0 = signals::PRESSURE_CHANNELS[BAROMETER_0.index()]
        .subscriber()
        .expect("too many subs on PRESSURE_CHANNELS; increase SUBS");
    let mut sub1 = signals::PRESSURE_CHANNELS[BAROMETER_1.index()]
        .subscriber()
        .expect("too many subs on PRESSURE_CHANNELS; increase SUBS");

    let mut filter = Ema::<f32>::new();
    let mut last_ts: Option<Instant> = None;
    let sender = signals::PRESSURE_WATCH.sender();

    loop {
        let sample: BaroSample =
            match select(sub0.next_message_pure(), sub1.next_message_pure()).await {
                Either::First(s) | Either::Second(s) => s,
            };

        let dt = last_ts
            .map(|prev| sample.ts.saturating_duration_since(prev).as_micros() as f32 / 1e6)
            .unwrap_or(FALLBACK_DT_S);
        last_ts = Some(sample.ts);

        let alpha = dt / (PRESSURE_TAU_S + dt);
        let filtered = filter.update_with_alpha(sample.pressure_mbar, alpha);

        let out = Pressure {
            ts: sample.ts,
            mbar: filtered,
        };
        sender.send(out);
        trace!("pressure: filtered={} mbar", filtered);
    }
}
