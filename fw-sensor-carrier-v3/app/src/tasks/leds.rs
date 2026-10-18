//! Board LED patterns. Green shows startup and liveness, yellow shows SD
//! logging, and red shows whether this is a development build.

use embassy_futures::join::join3;
use embassy_stm32::gpio::{Level, Output};
use embassy_time::{Duration, Timer, with_timeout};

use crate::resources::leds::BoardLeds;
use crate::signals;

#[embassy_executor::task]
pub async fn task(leds: BoardLeds) {
    join3(
        heartbeat(leds.green),
        sd_status(leds.yellow),
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
        Timer::after_millis(750).await;
        green.set_low();
        Timer::after_millis(250).await;
    }
}

/// On after a successful SD flush; off during initialization or retries.
async fn sd_status(mut yellow: Output<'static>) {
    let mut logging = signals::SD_LOGGING
        .receiver()
        .expect("LEDs: SD status receiver available");

    yellow.set_low();
    loop {
        yellow.set_level(if logging.changed().await {
            Level::High
        } else {
            Level::Low
        });
    }
}

async fn build_status(mut red: Output<'static>) {
    let warning_build = crate::built::GIT_DIRTY.unwrap_or(false)
        || crate::built::PROFILE != "release"
        || crate::built::FEATURES_LOWERCASE.contains(&"debug");
    let (on_ms, off_ms) = if warning_build {
        (125, 125)
    } else {
        (1800, 200)
    };

    loop {
        red.set_high();
        Timer::after_millis(on_ms).await;
        red.set_low();
        Timer::after_millis(off_ms).await;
    }
}
