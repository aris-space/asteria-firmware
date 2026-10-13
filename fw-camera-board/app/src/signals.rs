use datatypes::status::ArmingState;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;

/// The flight computer's software arming state, from its 1 Hz status message.
pub static FC_ARMING: Watch<CriticalSectionRawMutex, ArmingState, 1> = Watch::new();
