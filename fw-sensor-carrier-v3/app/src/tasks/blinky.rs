use defmt::info;
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn run() -> ! {
    info!("blink task: acquired all resources");

    loop {
        //yellow.set_high();
        Timer::after(Duration::from_millis(500)).await;
        //yellow.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}
