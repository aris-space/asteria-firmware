//! Task modules.
//!
//! Sensor tasks follow the generic-inner / concrete-wrapper pattern:
//!   1. A generic `run_inner(...)` with embedded-hal trait bounds (board-agnostic).
//!   2. A `#[embassy_executor::task]` wrapper `task(...)` with concrete types that delegates to it.

use embassy_time::Duration;

pub mod blinky;
pub mod logger;
pub mod readout;

pub const MAX_CONSECUTIVE_ERRORS: u8 = 10;
const BASE_BACKOFF_MS: u64 = 100;
const MAX_BACKOFF_MS: u64 = 5000;

pub fn backoff(attempt: u8) -> Duration {
    let ms = (BASE_BACKOFF_MS << attempt.min(6)).min(MAX_BACKOFF_MS);
    Duration::from_millis(ms)
}
