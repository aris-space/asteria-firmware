use crate::valves::{ExternalValve, OnboardValve};
use crate::valves::{EXTERNAL_VALVE_CONTROL, FSS_MAIN_CONTROL, OSS_MAIN_CONTROL};
use embassy_sync::pubsub::PubSubBehavior;

pub async fn actuate_onboard_valve(valve: OnboardValve) {
    match valve {
        // Onboard valves
        OnboardValve::FuelMain(state) => {
            FSS_MAIN_CONTROL.sender().send(state);
        }
        OnboardValve::OxidizerMain(state) => {
            OSS_MAIN_CONTROL.sender().send(state);
        }
    }
}

pub async fn actuate_external_valve(valve: ExternalValve) {
    EXTERNAL_VALVE_CONTROL.publish_immediate(valve);
}
