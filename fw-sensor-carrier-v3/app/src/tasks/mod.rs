//! Task modules.
//!
//! Sensor tasks follow the generic-inner / concrete-wrapper pattern:
//!   1. A generic `run_inner(...)` with embedded-hal trait bounds (board-agnostic).
//!   2. A `#[embassy_executor::task]` wrapper `task(...)` with concrete types that delegates to it.

pub mod barometer;
pub mod blinky;
pub mod imu;
