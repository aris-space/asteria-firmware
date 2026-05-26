//! Per-sensor calibration. Every submodule exposes `apply_calibration`, which
//! turns a raw sample into a calibrated one using values it loads internally.
//! `baro`/`dht`/`gnss`/`imu` only apply a fixed read-latency offset (the `imu`
//! also applies a fixed sensor-to-board axis remap); `mag` is flash-backed: it
//! `load`s a stored cal at boot and exposes caller-driven calibration state that
//! measures and persists a new one.

use core::fmt;

use serde::{Deserialize, Serialize};

pub mod baro;
pub mod dht;
pub mod gnss;
pub mod imu;
pub mod mag;

/// Fixed-capacity, zero-padded user label persisted with a flash-backed cal.
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Name([u8; Name::CAP]);

impl Name {
    pub const CAP: usize = 16;

    pub const fn new(s: &str) -> Self {
        let b = s.as_bytes();
        let mut out = [0u8; Self::CAP];
        let mut i = 0;
        while i < b.len() && i < Self::CAP {
            out[i] = b[i];
            i += 1;
        }
        Self(out)
    }

    pub fn as_str(&self) -> &str {
        let len = self.0.iter().position(|&c| c == 0).unwrap_or(Self::CAP);
        core::str::from_utf8(&self.0[..len]).unwrap_or("?")
    }
}

impl fmt::Display for Name {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

impl defmt::Format for Name {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}", self.as_str());
    }
}
