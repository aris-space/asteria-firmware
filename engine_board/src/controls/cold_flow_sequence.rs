#![allow(dead_code)]
use crate::controls::actions::Actions;
use crate::controls::actions::Actions::*;
use crate::valves::{ExternalValve::*, OnboardValve::*};
use embassy_time::Duration;
use hermes_can::messages::board_status::ValveState::*;
use hermes_can::messages::event_messages::DprState::Disabled;

// Firing Sequence
// assumes feed pressures are already set
#[allow(dead_code)]
pub static COLD_FLOW_SEQUENCE: [Actions; 9] = [
    //ActuateOnboard(FuelMain(Active)), // Open fuel main valve
    ActuateOnboard(OxidizerMain(Active)), // Open oxidizer main valve
    // ActuateExternal(FuelDpr(Enabled(15.2))),

    // wait for 0.5 seconds
    Wait(Duration::from_millis(5000)),
    // Follow desired thrust curve
    // Engine Shutdown
    // Shut down oxidizer side
    ActuateOnboard(OxidizerMain(Inactive)),
    //Wait(Duration::from_millis(250)),

    // Shut down fuel side
    ActuateOnboard(FuelMain(Inactive)),
    Wait(Duration::from_millis(250)),
    // Disable DPRs and vent tanks
    ActuateExternal(FuelDpr(Disabled)),
    ActuateExternal(OxidizerDpr(Disabled)),
    ActuateExternal(FuelVent(Inactive)),   // NO valve
    ActuateExternal(OxidizerVent(Active)), // NC valve
];
