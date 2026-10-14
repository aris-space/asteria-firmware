use defmt::info;
use embassy_stm32::gpio::Output;
use embassy_time::Timer;

use crate::resources::Address;

pub async fn led(led: &mut Output<'static>) {
    info!("led: debug LED on");
    led.set_high();
    Timer::after_millis(500).await;
}

pub async fn address(address: Address) {
    let address = address.read().await;
    info!("address: {} (0b{:04b})", address, address);
}

pub async fn camera(mut power: Output<'static>) -> ! {
    info!("camera: power on, check the green camera power LED and that it records");
    power.set_high();
    // TODO: untested on hw
    // crate::runcam::check(camera_uart).await;
    loop {
        core::future::pending::<()>().await;
    }
}
