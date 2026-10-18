//! Board LED patterns. Green shows startup and liveness, yellow shows SD
//! logging, and red shows whether this is a development build. Green after
//! startup and red are the standard `embedded_utils::led` tasks.

use embassy_executor::SendSpawner;
use embassy_stm32::gpio::{Level, Output};
use embassy_time::{Duration, Timer, with_timeout};

use crate::signals;

/// Fast blink until startup completes, then hands the LED to the standard heartbeat.
#[embassy_executor::task]
pub async fn startup(mut green: Output<'static>, spawner: SendSpawner) {
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

    spawner.spawn(
        embedded_utils::led::heartbeat(green, Level::High).expect("Failed to spawn heartbeat"),
    );
}

/// On after a successful SD flush; off during initialization or retries.
#[embassy_executor::task]
pub async fn sd_status(mut yellow: Output<'static>) {
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
