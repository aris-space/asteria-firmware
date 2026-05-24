//! Per-sensor calibration. Every submodule exposes `apply_calibration`, which
//! turns a raw sample into a calibrated one using values it loads internally.
//! `baro`/`dht`/`gnss` only apply a fixed read-latency offset; `mag`/`imu` are
//! flash-backed: they `load` a stored cal at boot and expose a `run` routine
//! (invoked from the console) that measures and persists a new one.

pub mod baro;
pub mod dht;
pub mod gnss;
pub mod imu;
mod imu_fit;
pub mod mag;
