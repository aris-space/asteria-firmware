//! Per-sensor calibration. Each submodule exposes `apply_calibration`,
//! which loads the relevant calibration values internally and returns
//! a calibrated sample. The loading source can later move from `const`
//! tables to flash-backed params without touching call sites.

pub mod baro;
pub mod dht;
pub mod gnss;
pub mod imu;
pub mod mag;
