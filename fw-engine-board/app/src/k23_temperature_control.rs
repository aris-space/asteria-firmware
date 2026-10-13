use crate::globals::STATE;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_stm32::gpio::Output;
use embassy_time::Timer;
use embedded_utils::info;

const LOWER_TEMP_THRESHOLD: f32 = 15.0; // Degrees Celsius
const UPPER_TEMP_THRESHOLD: f32 = 30.0; // Degrees Celsius

pub(crate) static HEATING_CONTROL_ACTIVE: AtomicBool = AtomicBool::new(true);

#[embassy_executor::task]
pub async fn k23_temperature_control(mut pin: Output<'static>) {
    let mut fss_inj_t_watcher = STATE.fss_inj_t.receiver().unwrap();
    let mut oss_tnk_t_watcher = STATE.oss_tnk_t.receiver().unwrap();

    loop {
        if !HEATING_CONTROL_ACTIVE.load(Ordering::Relaxed) {
            pin.set_low();
            Timer::after_millis(2000).await;
            continue;
        }

        let fss_inj_t = fss_inj_t_watcher.get().await;
        let oss_tnk_t = oss_tnk_t_watcher.get().await;

        if (fss_inj_t.0 < LOWER_TEMP_THRESHOLD) | (oss_tnk_t.0 < LOWER_TEMP_THRESHOLD) {
            pin.set_high();
            info!("Heater turned ON");
        } else if (fss_inj_t.0 > UPPER_TEMP_THRESHOLD) | (oss_tnk_t.0 > UPPER_TEMP_THRESHOLD) {
            pin.set_low();
            info!("Heater turned OFF");
        }
        Timer::after_millis(1000).await;
    }
}
