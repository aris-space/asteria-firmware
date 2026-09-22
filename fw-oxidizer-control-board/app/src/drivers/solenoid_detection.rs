// TODO: PC2 and PC3 are placeholder pins.
use crate::globals::STATE;
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::peripherals::{PB6, PC2};
use embassy_time::{Duration, Ticker};
use embedded_utils::info;

#[derive(Clone, Copy)]
#[allow(dead_code)] // Temporary until this is consumed by the CAN implementation.
pub struct SolenoidStates {
    pub dpr: bool,
    pub oxidizer_vent: bool,
}

#[embassy_executor::task]
pub async fn solenoid_detection_task(
    dpr_detect: Peri<'static, PC2>,
    oxidizer_vent_detect: Peri<'static, PB6>,
) -> ! {
    let dpr_pin = Input::new(dpr_detect, Pull::Down);
    let oxidizer_vent_pin = Input::new(oxidizer_vent_detect, Pull::Down);

    let sender = STATE.solenoid_states.sender();
    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        sender.send(SolenoidStates {
            dpr: dpr_pin.is_high(),
            oxidizer_vent: oxidizer_vent_pin.is_high(),
        });
        info!("DPR: {}", dpr_pin.is_high());
        info!("Oxidizer vent: {}", oxidizer_vent_pin.is_high());
        ticker.next().await;
    }
}
