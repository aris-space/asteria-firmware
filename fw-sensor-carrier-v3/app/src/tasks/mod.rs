//! Tasks.
//!
//! Readouts (`readout`) drive individual sensors and publish raw samples to
//! per-sensor signals. Processing tasks (`processing`) consume raw samples and
//! publish derived/fused signals. CAN (`can`) transmits derived signals to the
//! bus and listens for control frames.

use embassy_time::Duration;

pub mod blinky;
pub mod can;
pub mod processing;
pub mod readout;

pub const MAX_CONSECUTIVE_ERRORS: u8 = 10;
const BASE_BACKOFF_MS: u64 = 100;
const MAX_BACKOFF_MS: u64 = 5000;

pub fn backoff(attempt: u8) -> Duration {
    let ms = (BASE_BACKOFF_MS << attempt.min(6)).min(MAX_BACKOFF_MS);
    Duration::from_millis(ms)
}
