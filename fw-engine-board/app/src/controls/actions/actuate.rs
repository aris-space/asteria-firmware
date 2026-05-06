use crate::globals::STATE;
use crate::valves::OnboardValve;

pub async fn actuate_onboard_valve(valve: OnboardValve) {
    match valve {
        // Onboard valves
        OnboardValve::FuelMain(state) => {
            STATE.fuel_main_control.sender().send(state);
        }
        OnboardValve::OxidizerMain(state) => {
            STATE.oxidizer_main_control.sender().send(state);
        }
    }
}
