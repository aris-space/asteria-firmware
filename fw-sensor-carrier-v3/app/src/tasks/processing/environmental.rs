use defmt::trace;
use embassy_futures::select::{Either4, select4};
use embassy_time::Instant;

use super::filters::Ema;
use crate::measurements::Environment;
use crate::sensors::{BAROMETER_0, BAROMETER_1, DHT_0, DHT_1};
use crate::signals;

const PRESSURE_TAU_S: f32 = 120.0;
const TH_TAU_S: f32 = 20.0;

const FALLBACK_PRESSURE_DT_S: f32 = 1.0 / 40.0;
const FALLBACK_TH_DT_S: f32 = 1.0;

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut p0 = signals::PRESSURE_CHANNELS[BAROMETER_0.index()]
        .subscriber()
        .expect("too many subs on PRESSURE_CHANNELS; increase SUBS");
    let mut p1 = signals::PRESSURE_CHANNELS[BAROMETER_1.index()]
        .subscriber()
        .expect("too many subs on PRESSURE_CHANNELS; increase SUBS");
    let mut e0 = signals::ENV_CHANNELS[DHT_0.index()]
        .subscriber()
        .expect("too many subs on ENV_CHANNELS; increase SUBS");
    let mut e1 = signals::ENV_CHANNELS[DHT_1.index()]
        .subscriber()
        .expect("too many subs on ENV_CHANNELS; increase SUBS");

    let mut pressure_filter = Ema::<f32>::new();
    let mut temp_filter = Ema::<f32>::new();
    let mut hum_filter = Ema::<f32>::new();

    let mut last_pressure_ts: Option<Instant> = None;
    let mut last_th_ts: Option<Instant> = None;

    let sender = signals::ENVIRONMENT_WATCH.sender();

    loop {
        let latest_ts = match select4(
            p0.next_message_pure(),
            p1.next_message_pure(),
            e0.next_message_pure(),
            e1.next_message_pure(),
        )
        .await
        {
            Either4::First(p) | Either4::Second(p) => {
                let ts = p.ts;
                let dt = last_pressure_ts
                    .map(|prev| ts.saturating_duration_since(prev).as_micros() as f32 / 1e6)
                    .unwrap_or(FALLBACK_PRESSURE_DT_S);
                last_pressure_ts = Some(ts);
                let alpha = dt / (PRESSURE_TAU_S + dt);
                pressure_filter.update_with_alpha(p.pressure_mbar, alpha);
                ts
            }
            Either4::Third(env) | Either4::Fourth(env) => {
                let ts = env.ts;
                let dt = last_th_ts
                    .map(|prev| ts.saturating_duration_since(prev).as_micros() as f32 / 1e6)
                    .unwrap_or(FALLBACK_TH_DT_S);
                last_th_ts = Some(ts);
                let alpha = dt / (TH_TAU_S + dt);
                temp_filter.update_with_alpha(env.temperature_c, alpha);
                hum_filter.update_with_alpha(env.humidity_rh, alpha);
                ts
            }
        };

        let fused = Environment {
            ts: latest_ts,
            temperature_c: temp_filter.current_value(),
            humidity_rh: hum_filter.current_value(),
            pressure_mbar: pressure_filter.current_value(),
        };
        sender.send(fused);
        trace!(
            "env: t={} c rh={} % p={} mbar",
            fused.temperature_c, fused.humidity_rh, fused.pressure_mbar
        );
    }
}
