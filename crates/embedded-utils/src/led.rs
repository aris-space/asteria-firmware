//! Standard board LED tasks, see "Board LEDs" in docs/ARCHITECTURE.md.
//!
//! `on_level` is the pin level at which the LED lights.

use datatypes::status::BuildInformationCommon;
use embassy_stm32::gpio::{Level, Output};
use embassy_time::Timer;

async fn blink(led: &mut Output<'static>, on_level: Level, on_ms: u64, off_ms: u64) -> ! {
    let off = match on_level {
        Level::High => Level::Low,
        Level::Low => Level::High,
    };
    loop {
        led.set_level(on_level);
        Timer::after_millis(on_ms).await;
        led.set_level(off);
        Timer::after_millis(off_ms).await;
    }
}

/// Green heartbeat LED: 750 ms on, 250 ms off (75% duty).
#[embassy_executor::task]
pub async fn heartbeat(mut led: Output<'static>, on_level: Level) -> ! {
    blink(&mut led, on_level, 750, 250).await
}

/// Red build status LED.
///
/// - clean release build: 1800 ms on, 200 ms off (90% duty)
/// - dirty tree, non-release profile, or defmt over RTT enabled: 125 ms on, 125 ms off
#[embassy_executor::task]
pub async fn build_status(
    mut led: Output<'static>,
    build_info: &'static BuildInformationCommon,
    on_level: Level,
) -> ! {
    let warning_build =
        build_info.is_git_dirty || !build_info.is_release || build_info.debug_defmt_rtt;
    let (on_ms, off_ms) = if warning_build {
        (125, 125)
    } else {
        (1800, 200)
    };
    blink(&mut led, on_level, on_ms, off_ms).await
}
