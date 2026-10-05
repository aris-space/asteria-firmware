use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Instant, Ticker, Timer};

use crate::signals;

static STARTUP_COMPLETE: AtomicBool = AtomicBool::new(false);
const ESTIMATE_TIMEOUT: Duration = Duration::from_millis(500);

pub fn mark_startup_complete() {
    STARTUP_COMPLETE.store(true, Ordering::Relaxed);
}

/// Off without a fresh estimate, blinking before MSL anchoring, solid afterward.
#[embassy_executor::task]
pub async fn estimate_status(mut yellow: Output<'static>) -> ! {
    let mut estimates = signals::STATE_ESTIMATE_WATCH
        .receiver()
        .expect("yellow LED: estimate receiver available");
    let mut last_estimate = None;
    let mut msl_ready = false;
    let mut ticker = Ticker::every(Duration::from_millis(50));

    loop {
        ticker.next().await;
        let now = Instant::now();
        if let Some(estimate) = estimates.try_changed() {
            last_estimate = Some(now);
            msl_ready = estimate.msl_ready;
        }
        let fresh = last_estimate
            .is_some_and(|last| now.saturating_duration_since(last) <= ESTIMATE_TIMEOUT);
        let on = fresh && (msl_ready || now.as_millis() % 1_000 < 250);
        if on {
            yellow.set_high();
        } else {
            yellow.set_low();
        }
    }
}

#[embassy_executor::task]
pub async fn heartbeat(mut green: Output<'static>) -> ! {
    loop {
        green.set_high();
        Timer::after_millis(250).await;
        green.set_low();
        let off_ms = if STARTUP_COMPLETE.load(Ordering::Relaxed) {
            750
        } else {
            250
        };
        Timer::after_millis(off_ms).await;
    }
}

#[embassy_executor::task]
pub async fn build_status(mut red: Output<'static>, warning_build: bool) -> ! {
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
