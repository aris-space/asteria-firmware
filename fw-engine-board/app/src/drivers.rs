use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;

pub(crate) mod pressure;
pub mod solenoid_detection;
pub(crate) mod temperature;

pub const WATCH: usize = 4;

pub static ENGINE_P_WATCH: Watch<ThreadModeRawMutex, f32, WATCH> = Watch::new();
