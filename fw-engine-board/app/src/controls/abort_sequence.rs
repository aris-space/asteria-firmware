use crate::controls::actions::Actions;
use crate::controls::actions::Actions::*;
use crate::valves::OnboardValve::{FuelMain, OxidizerMain};
use datatypes::actuator::NormallyClosedValve::Closed;

// Firing Abort Sequence
pub static ABORT_SEQUENCE: [Actions; 2] = [
    ActuateOnboard(FuelMain(Closed)),
    ActuateOnboard(OxidizerMain(Closed)),
];
