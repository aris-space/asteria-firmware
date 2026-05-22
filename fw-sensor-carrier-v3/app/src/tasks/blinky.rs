use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

const HALF_PERIOD: Duration = Duration::from_millis(500);

#[embassy_executor::task]
pub async fn task(mut led: Output<'static>) -> ! {
    loop {
        led.set_high();
        Timer::after(HALF_PERIOD).await;
        led.set_low();
        Timer::after(HALF_PERIOD).await;
    }
}
