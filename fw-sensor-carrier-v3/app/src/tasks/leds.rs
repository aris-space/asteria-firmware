//! Board LED patterns. Green shows startup and liveness, yellow shows estimate
//! readiness, and red shows whether this is a development build.

use embassy_futures::join::join3;
use embassy_stm32::gpio::{Level, Output};
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};

use crate::resources::leds::BoardLeds;
use crate::signals;

const ESTIMATE_TIMEOUT: Duration = Duration::from_millis(500);

#[embassy_executor::task]
pub async fn task(leds: BoardLeds) {
    join3(
        heartbeat(leds.green),
        estimate_status(leds.yellow),
        build_status(leds.red),
    )
    .await;
}

async fn heartbeat(mut green: Output<'static>) {
    loop {
        green.set_high();
        Timer::after_millis(250).await;
        green.set_low();

        if with_timeout(Duration::from_millis(250), signals::STARTUP_COMPLETE.wait())
            .await
            .is_ok()
        {
            break;
        }
    }

    loop {
        green.set_high();
        Timer::after_millis(250).await;
        green.set_low();
        Timer::after_millis(750).await;
    }
}

/// Off without a fresh estimate, blinking before MSL anchoring, solid afterward.
async fn estimate_status(mut yellow: Output<'static>) {
    let mut estimates = signals::STATE_ESTIMATE_WATCH
        .receiver()
        .expect("LEDs: estimate receiver available");
    let mut last_estimate = None;
    let mut msl_ready = false;
    let mut blink_phase = 0;
    let mut ticker = Ticker::every(Duration::from_millis(250));

    loop {
        ticker.next().await;
        let now = Instant::now();
        if let Some(estimate) = estimates.try_changed() {
            last_estimate = Some(now);
            msl_ready = estimate.msl_ready;
        }
        // `msl_ready` stays true after GNSS loss. Solid yellow means a fresh
        // estimate with an established MSL reference, not a live GNSS fix.
        let fresh = last_estimate
            .is_some_and(|last| now.saturating_duration_since(last) <= ESTIMATE_TIMEOUT);
        yellow.set_level(if fresh && (msl_ready || blink_phase == 0) {
            Level::High
        } else {
            Level::Low
        });
        blink_phase = (blink_phase + 1) % 4;
    }
}

async fn build_status(mut red: Output<'static>) {
    let warning_build = crate::built::GIT_DIRTY.unwrap_or(false)
        || crate::built::PROFILE != "release"
        || crate::built::FEATURES_LOWERCASE.contains(&"debug");
    let (on_ms, off_ms) = if warning_build {
        (125, 125)
    } else {
        (200, 1800)
    };

    loop {
        red.set_high();
        Timer::after_millis(on_ms).await;
        red.set_low();
        Timer::after_millis(off_ms).await;
    }
}
