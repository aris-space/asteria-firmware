use datatypes::status::ArmingState;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;

pub static FC_ARMING: Watch<CriticalSectionRawMutex, ArmingState, 1> = Watch::new();
