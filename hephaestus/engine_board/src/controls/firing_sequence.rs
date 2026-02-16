use crate::controls::actions::Actions::*;
use crate::controls::actions::{Actions, Detection, Watcher};
use crate::sensors::Sensor::*;
use crate::valves::{ExternalValve::*, OnboardValve::*};
use embassy_time::Duration;
use hermes_can::messages::board_status::ValveState::*;
use hermes_can::messages::event_messages::DprState::{Disabled, Enabled};

// Firing Sequence
// assumes upstream pressures are already set
#[allow(dead_code)]
pub static FIRING_SEQUENCE: [Actions; 30] = [
    // Make sure DPRs are on the correct setpoint
    ActuateExternal(FuelDpr(Enabled(17.01))),
    ActuateExternal(OxidizerDpr(Enabled(19.27))),
    // Initiate Igniter
    // Assuming Igniter is Purging
    ActuateExternal(IgniterPurge(Inactive)), // Stop purging
    ActuateExternal(IgniterSpark(Active)),   // Start spark and wait shortly
    Wait(Duration::from_millis(1500)),
    ActuateExternal(IgniterOxidizer(Active)), // Open igniter oxidizer and wait shortly
    Wait(Duration::from_millis(500)),
    ActuateExternal(IgniterFuel(Active)), // Open igniter fuel
    // Wait for stable ignition by checking igniter chamber pressure
    Detect(Detection {
        sensor: IgniterP(4.0),                 // Target pressure of 4.0 bar
        increasing: true,                      // Expecting pressure to increase
        patience: Duration::from_millis(3000), // Patience of 3.0 s
    }),
    // Ensure igniter pressure doesn't drop below threshold during startup
    Watch(Watcher {
        sensor: IgniterP(4.0), // Threshold to ensure pressure doesn't drop below 4.0 bar
        increasing: false,
    }),
    // Igniter is stable, initiate main flow
    ActuateOnboard(OxidizerMain(Active)), // Open oxidizer main valve
    Wait(Duration::from_millis(300)),     // Wait for oxidizer to start flowing
    ActuateOnboard(FuelMain(Active)),     // Open fuel main valve
    // Initiate early ramp up
    ActuateExternal(FuelDpr(Enabled(31.41))), // middle between startup and full thrust
    ActuateExternal(OxidizerDpr(Enabled(34.64))), // middle between startup and full thrust
    // Wait for stable combustion by checking engine chamber pressure
    Detect(Detection {
        sensor: EngineP(6.5), // Target pressure of 6.5 bar, ~ half of ignition cc pressure
        increasing: true,     // Expecting pressure to increase
        patience: Duration::from_millis(1000), // Patience of 1.0 s
    }),
    // Ignore Igniter pressure
    Ignore(IgniterP(4.0)),
    // Full Throttle
    ActuateExternal(FuelDpr(Enabled(45.81))),
    ActuateExternal(OxidizerDpr(Enabled(50.0))),
    // for 5s
    Wait(Duration::from_millis(5000)),
    // Throttle down
    ActuateExternal(FuelDpr(Enabled(18.63))),
    ActuateExternal(OxidizerDpr(Enabled(20.58))),
    // for 20s, making sure tanks are emptied
    Wait(Duration::from_millis(25000)),
    // Engine Shutdown
    ActuateOnboard(OxidizerMain(Inactive)),
    ActuateOnboard(FuelMain(Inactive)),
    ActuateExternal(FuelDpr(Disabled)),
    ActuateExternal(OxidizerDpr(Disabled)),
    ActuateExternal(FuelVent(Inactive)),     // NO valve
    ActuateExternal(OxidizerVent(Active)),   // NC valve
    ActuateExternal(NitrogenVent(Inactive)), // NO valve
];
