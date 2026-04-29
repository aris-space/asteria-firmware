use embassy_stm32::gpio::{Input, Output};
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn power_indicators(
    bat_p: Input<'static>,
    ext_p: Input<'static>,
    mut led_bat_p: Output<'static>,
    mut led_ext_p: Output<'static>,
) {
    loop {
        led_bat_p.set_level(bat_p.get_level());
        led_ext_p.set_level(ext_p.get_level());
        Timer::after(Duration::from_millis(100)).await;
    }
}
