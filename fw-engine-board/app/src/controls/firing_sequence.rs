use crate::controls::actions::Actions::*;
use crate::controls::actions::{Actions, Detection};
use crate::sensors::Sensor::*;
use crate::valves::OnboardValve::*;
use datatypes::actuator::NormallyClosedValve::{Closed, Open};
use embassy_time::Duration;

// Firing Sequence
// External feed system valves and DPRs are now handled by their own control boards.
#[allow(dead_code)]
pub static FIRING_SEQUENCE: [Actions; 10] = [
    // Initiate main flow. Igniter sequencing is external to this board now.
    ActuateOnboard(OxidizerMain(Open)),
    Wait(Duration::from_millis(300)),
    ActuateOnboard(FuelMain(Open)),
    // Wait for stable combustion by checking engine chamber pressure.
    Detect(Detection {
        sensor: EngineP(6.5),
        increasing: true,
        patience: Duration::from_millis(1000),
    }),
    // Full throttle dwell.
    Wait(Duration::from_millis(5000)),
    // Throttle down / tail-off dwell.
    Wait(Duration::from_millis(25000)),
    // Engine shutdown.
    ActuateOnboard(OxidizerMain(Closed)),
    ActuateOnboard(FuelMain(Closed)),
    Ignore(EngineP(0.0)),
    Wait(Duration::from_millis(250)),
];
