use crate::controls::actions::Actions;
use crate::controls::actions::Actions::*;
use crate::valves::ExternalValve::{FuelDpr, FuelVent, NitrogenVent, OxidizerDpr, OxidizerVent};
use crate::valves::OnboardValve::{FuelMain, OxidizerMain};
use hermes_can::messages::board_status::ValveState::{Active, Inactive};
use hermes_can::messages::event_messages::DprState::Disabled;

// Firing Abort Sequence
pub static ABORT_SEQUENCE: [Actions; 7] = [
    ActuateOnboard(FuelMain(Inactive)),
    ActuateOnboard(OxidizerMain(Inactive)),
    ActuateExternal(FuelDpr(Disabled)),
    ActuateExternal(OxidizerDpr(Disabled)),
    ActuateExternal(FuelVent(Inactive)),     // NO valve
    ActuateExternal(OxidizerVent(Active)),   // NC valve
    ActuateExternal(NitrogenVent(Inactive)), // NO valve
];
