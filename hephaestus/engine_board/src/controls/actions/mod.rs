#![allow(dead_code)]

pub mod actuate;
pub mod detect;
pub mod follow_thrust_curve;
pub mod wait;
pub mod watch;

use crate::sensors::Sensor;
use crate::valves::{ExternalValve, OnboardValve};
use embassy_time::Duration;

#[derive(PartialEq)]
pub enum ActionCompleteness {
    Successful,
    Failed,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Actions {
    Wait(Duration),
    ActuateExternal(ExternalValve),
    ActuateOnboard(OnboardValve),
    Detect(Detection),
    Watch(Watcher),
    Ignore(Sensor),
    FollowThrustCurve,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Detection {
    pub sensor: Sensor,
    pub increasing: bool,
    pub patience: Duration,
}

#[derive(Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Watcher {
    pub(crate) sensor: Sensor,
    pub(crate) increasing: bool,
}
