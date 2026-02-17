#![no_std]

pub mod fmt;

use core::sync::atomic::AtomicU32;
use embassy_time::{Duration, Instant};

pub static ERROR_COUNT: AtomicU32 = AtomicU32::new(0);
pub static WARN_COUNT: AtomicU32 = AtomicU32::new(0);

pub trait ExtendTime {
    fn as_secs_f32(&self) -> f32;
}
impl ExtendTime for Instant {
    fn as_secs_f32(&self) -> f32 {
        self.as_micros() as f32 / 1e6
    }
}
impl ExtendTime for Duration {
    fn as_secs_f32(&self) -> f32 {
        self.as_micros() as f32 / 1e6
    }
}
