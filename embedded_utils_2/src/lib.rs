#![no_std]

pub mod fmt;

use core::sync::atomic::AtomicU32;
use embassy_stm32::time::Hertz;
use embassy_stm32::Config;
use embassy_time::{Duration, Instant};

pub fn clocks_config() -> Config {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(16_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL20,
            divp: Some(PllPDiv::DIV2),
            divq: Some(PllQDiv::DIV2),
            divr: Some(PllRDiv::DIV2),
        });
    }
    config
}

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
