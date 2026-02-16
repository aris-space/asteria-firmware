use crate::drivers::digital_pressure::DIGITAL_TEMPERATURE_WATCH;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::gpio::Output;
use embassy_time::Timer;
use embedded_utils::info;

const LOWER_TEMP_THRESHOLD: f32 = 15.0; // Degrees Celsius
const UPPER_TEMP_THRESHOLD: f32 = 30.0; // Degrees Celsius

pub(crate) static HEATING_CONTROL_ACTIVE: AtomicBool = AtomicBool::new(true);

#[embassy_executor::task]
pub async fn k23_temperature_control(mut pin: Output<'static>) {
    let mut temperature_watcher = DIGITAL_TEMPERATURE_WATCH.receiver().unwrap();

    loop {
        if !HEATING_CONTROL_ACTIVE.load(Ordering::Relaxed) {
            pin.set_low();
            Timer::after_millis(2000).await;
            continue;
        }

        let temp = temperature_watcher.get().await;

        if (temp.eng_cc_t < LOWER_TEMP_THRESHOLD)
            | (temp.fue_inj_t < LOWER_TEMP_THRESHOLD)
            | (temp.oxd_inj_t < LOWER_TEMP_THRESHOLD)
        {
            pin.set_high();
            info!("Heater turned ON");
        } else if (temp.eng_cc_t > UPPER_TEMP_THRESHOLD)
            | (temp.fue_inj_t > UPPER_TEMP_THRESHOLD)
            | (temp.oxd_inj_t > UPPER_TEMP_THRESHOLD)
        {
            pin.set_low();
            info!("Heater turned OFF");
        }
        Timer::after_millis(1000).await;
    }
}
