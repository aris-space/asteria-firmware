// TODO: PC2, PC3, and PA1 are placeholder pins.
use crate::globals::STATE;
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::peripherals::{PA1, PC2, PC3};
use embassy_time::{Duration, Ticker};
use embedded_utils::info;

#[derive(Clone, Copy)]
#[allow(dead_code)] // Temporary until this is consumed by the CAN implementation.
pub struct SolenoidStates {
    pub fuel_main: bool,
    pub oxidizer_main: bool,
    pub heating_pad: bool,
}

#[embassy_executor::task]
pub async fn solenoid_detection_task(
    fuel_main_detect: Peri<'static, PC2>,
    oxidizer_main_detect: Peri<'static, PC3>,
    heating_pad_detect: Peri<'static, PA1>,
) -> ! {
    let fuel_main_pin = Input::new(fuel_main_detect, Pull::Down);
    let oxidizer_main_pin = Input::new(oxidizer_main_detect, Pull::Down);
    let heating_pad_pin = Input::new(heating_pad_detect, Pull::Down);

    let sender = STATE.solenoid_states.sender();
    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        sender.send(SolenoidStates {
            fuel_main: fuel_main_pin.is_high(),
            oxidizer_main: oxidizer_main_pin.is_high(),
            heating_pad: heating_pad_pin.is_high(),
        });
        info!("Fuel main: {}", fuel_main_pin.is_high());
        info!("Oxidizer main: {}", oxidizer_main_pin.is_high());
        info!("Heating pad: {}", heating_pad_pin.is_high());
        ticker.next().await;
    }
}
