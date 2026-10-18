use crate::modbus_server::{DEVICE_STATUS, DeviceStatus};
use core::sync::atomic::Ordering;
use embassy_stm32::gpio::Output;
use embassy_time::Timer;

fn initializing() -> bool {
    DeviceStatus::from_bits_truncate(DEVICE_STATUS.load(Ordering::Relaxed))
        .contains(DeviceStatus::INITIALIZING)
}

/// Green heartbeat LED, fast blink while the motor initializes, then 750 ms on, 250 ms off.
/// Motor errors only show up in the Modbus device status register.
#[embassy_executor::task]
pub async fn heartbeat(mut green: Output<'static>) -> ! {
    while initializing() {
        green.set_high();
        Timer::after_millis(250).await;
        green.set_low();
        Timer::after_millis(250).await;
    }

    loop {
        green.set_high();
        Timer::after_millis(750).await;
        green.set_low();
        Timer::after_millis(250).await;
    }
}
