//! Sensor readout tasks: drive a single sensor (or sensor-bus pair), publish
//! raw samples to its per-sensor signal, and report `SensorStatus`.
//!
//! I2C readouts initialize sequentially at startup with bounded retries, then
//! run a read task. Failed initialization or too many read errors disables the
//! sensor until reboot. IMU/GNSS readouts retain `Inactive`/`Active` states and
//! retry initialization indefinitely. Each readout updates its sensor status
//! at state transitions.

use embassy_time::Duration;

pub mod barometer;
pub mod dht;
pub mod gnss;
pub mod imu;
pub mod magnetometer;

/// Maximum consecutive read errors before an I2C readout disables itself,
/// or an IMU/GNSS readout transitions back to `Inactive` and retries init.
pub const MAX_CONSECUTIVE_ERRORS: u8 = 10;

/// How many times a readout retries init before it gives up, disables itself, and
/// stops touching the bus. A sensor that never answers (unpopulated or shorted
/// bus) would otherwise retry forever, and each failed attempt busy-blocks the
/// shared executor (embassy's async I2C doesn't yield while the bus is wedged),
/// starving the healthy sensors on the same bus.
pub const MAX_INIT_ATTEMPTS: u8 = 3;

// Exponential backoff between init retries. The bus readouts take only a couple of
// these before they disable (see `MAX_INIT_ATTEMPTS`); keeping the base short means
// an absent/shorted bus runs through its attempts and goes quiet within ~0.6 s,
// before the healthy sensors settle into steady-state reads. IMU/GNSS retry
// indefinitely on dedicated buses and rely on the cap.
const BASE_BACKOFF_MS: u64 = 100;
const MAX_BACKOFF_MS: u64 = 5000;

/// Exponential backoff for sensor init retries. Saturates at `MAX_BACKOFF_MS`.
pub fn backoff(attempt: u8) -> Duration {
    let ms = (BASE_BACKOFF_MS << attempt.min(6)).min(MAX_BACKOFF_MS);
    Duration::from_millis(ms)
}
