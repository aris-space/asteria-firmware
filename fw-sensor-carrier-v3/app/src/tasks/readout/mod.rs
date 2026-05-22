//! Sensor readout tasks: drive a single sensor (or sensor-bus pair), publish
//! raw samples to its per-sensor signal, and report `SensorStatus`.
//!
//! Every readout follows the same shape: an `Inactive` state that retries
//! init with exponential backoff, an `Active` state that loops on the
//! sensor's wakeup/interrupt, and `run_inner` which flips the
//! [`crate::sensors::*_STATUS`] table at each state transition.

use embassy_time::Duration;

pub mod barometer;
pub mod dht;
pub mod gnss;
pub mod imu;
pub mod magnetometer;

/// Maximum consecutive read errors before a readout transitions back to
/// `Inactive` and retries init.
pub const MAX_CONSECUTIVE_ERRORS: u8 = 10;

const BASE_BACKOFF_MS: u64 = 100;
const MAX_BACKOFF_MS: u64 = 5000;

/// Exponential backoff for sensor init retries. Saturates at `MAX_BACKOFF_MS`.
pub fn backoff(attempt: u8) -> Duration {
    let ms = (BASE_BACKOFF_MS << attempt.min(6)).min(MAX_BACKOFF_MS);
    Duration::from_millis(ms)
}
