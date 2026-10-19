use crate::buzzer::BuzzerState;
use crate::globals::STATE;
use datatypes::status::ArmingState;
use embassy_stm32::gpio::{Input, Level};
use embassy_time::{Duration, Timer};

pub const MAIN_ARMING_UPDATE_RATE: Duration = Duration::from_millis(1000);

#[embassy_executor::task]
pub async fn check_main_arming(pin: Input<'static>) {
    let buzzer_armed_sender = STATE.buzzer.sender();
    let arming_state_sender = STATE.arming_state.sender();
    loop {
        match pin.get_level() {
            Level::High => {
                arming_state_sender.send(ArmingState::Safe);
                buzzer_armed_sender.send(BuzzerState::Idle);
            }
            Level::Low => {
                arming_state_sender.send(ArmingState::Armed);
                buzzer_armed_sender.send(BuzzerState::Armed);
            }
        }
        Timer::after(MAIN_ARMING_UPDATE_RATE).await;
    }
}
