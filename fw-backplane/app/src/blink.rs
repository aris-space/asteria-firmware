use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

#[embassy_executor::task(pool_size = 3)]
pub async fn blink(mut led: Output<'static>) -> ! {
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(200)).await;
        led.set_low();
        Timer::after(Duration::from_millis(800)).await;
    }
}
