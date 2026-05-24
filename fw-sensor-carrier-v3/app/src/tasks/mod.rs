//! Tasks.
//!
//! Readouts (`readout`) drive individual sensors and publish raw samples to
//! per-sensor signals. Processing tasks (`processing`) consume raw samples and
//! publish derived/fused signals. CAN (`can`) transmits derived signals to the
//! bus and listens for control frames.

pub mod blinky;
pub mod can;
pub mod console;
pub mod processing;
pub mod readout;
