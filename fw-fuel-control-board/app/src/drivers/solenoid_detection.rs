// TODO: PA0, PA1, PA2 are placeholder pins!!
use crate::globals::STATE;
use embassy_stm32::Peri;
use embassy_stm32::gpio::{Input, Pull};
use embassy_stm32::peripherals::{PA0, PA1, PA2};
use embassy_time::{Duration, Ticker};

#[derive(Clone, Copy)]
#[allow(dead_code)] //temporary until I add a consumer in the CAN implementation
pub struct SolenoidStates {
    pub dpr: bool,
    pub pressurization_vent: bool,
    pub fuel_vent: bool,
}

#[embassy_executor::task]
pub async fn solenoid_detection_task(
    dpr_detect: Peri<'static, PA0>,       // TODO: change pin
    prz_vent_detect: Peri<'static, PA1>,  // TODO: change pin
    fuel_vent_detect: Peri<'static, PA2>, // TODO: change pin
) -> ! {
    let dpr_pin = Input::new(dpr_detect, Pull::Down);
    let prz_vent_pin = Input::new(prz_vent_detect, Pull::Down);
    let fuel_vent_pin = Input::new(fuel_vent_detect, Pull::Down);

    let sender = STATE.solenoid_states.sender();
    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        sender.send(SolenoidStates {
            dpr: dpr_pin.is_high(),
            pressurization_vent: prz_vent_pin.is_high(),
            fuel_vent: fuel_vent_pin.is_high(),
        });
        ticker.next().await;
    }
}
