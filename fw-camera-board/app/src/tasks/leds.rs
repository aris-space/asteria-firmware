//! Debug LED pattern: fast blinking marks a development build, a short blink
//! every two seconds a clean release build.

use embassy_stm32::gpio::Output;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn task(mut led: Output<'static>) {
    let warning_build = crate::built::GIT_DIRTY.unwrap_or(false)
        || crate::built::PROFILE != "release"
        || crate::built::FEATURES_LOWERCASE.contains(&"debug");
    let (on_ms, off_ms) = if warning_build {
        (125, 125)
    } else {
        (200, 1800)
    };

    loop {
        led.set_high();
        Timer::after_millis(on_ms).await;
        led.set_low();
        Timer::after_millis(off_ms).await;
    }
}
