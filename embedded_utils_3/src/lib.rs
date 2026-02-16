#![no_std]

pub mod fmt;

use core::sync::atomic::AtomicU32;
use embassy_stm32::rcc::mux::I2c1235sel;
use embassy_stm32::rcc::mux::I2c4sel;
use embassy_stm32::rcc::mux::Saisel;
use embassy_stm32::rcc::mux::Usart16910sel;
use embassy_stm32::rcc::mux::Usart234578sel;
use embassy_stm32::rcc::{AHBPrescaler, APBPrescaler, Hse, Pll, PllDiv, Sysclk, VoltageScale};
use embassy_stm32::rcc::{HseMode, PllMul, PllPreDiv, PllSource};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Instant};

pub fn clocks_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();

    // 16 MHz XTAL
    config.rcc.hse = Some(Hse {
        freq: Hertz(16_000_000),
        mode: HseMode::Oscillator,
    });

    config.rcc.voltage_scale = VoltageScale::Scale0; // GO INSANTLY FAST.

    // PLL1: VCO = 16 MHz * 30 = 480 MHz
    //   P = 480/2 = 240 MHz -> SYSCLK
    //   Q = 480/2 = 240 MHz -> SPI123 kernel (fast)
    //   R = 480/2 = 240 MHz (not used explicitly)
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL30,
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV2),
        divr: Some(PllDiv::DIV2),
    });
    config.rcc.sys = Sysclk::PLL1_P;

    // PLL3 used for I2C kernels. Choose neat 120 MHz.
    // VCO3 = 16 * 30 = 480 MHz, R = 480/4 = 120 MHz
    config.rcc.pll3 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL30,
        divp: Some(PllDiv::DIV4),
        divq: Some(PllDiv::DIV4),
        divr: Some(PllDiv::DIV4), // -> 120 MHz
    });

    // Buses (respect H7 limits: HCLK <= 240, PCLKx <= 120).
    config.rcc.d1c_pre = AHBPrescaler::DIV1; // HCLK 240 MHz
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV2; // 120 MHz
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 120 MHz
    config.rcc.apb3_pre = APBPrescaler::DIV2; // 120 MHz
    config.rcc.apb4_pre = APBPrescaler::DIV2; // 120 MHz

    // Kernel muxes
    config.rcc.mux.spi123sel = Saisel::PLL1_Q; // 240 MHz for SPI1/2/3
    config.rcc.mux.i2c1235sel = I2c1235sel::PLL3_R; // 120 MHz for I2C1/2/3
    config.rcc.mux.i2c4sel = I2c4sel::PLL3_R; // 120 MHz for I2C4/5

    // (optional, handy defaults)
    config.rcc.mux.usart16910sel = Usart16910sel::PCLK2;
    config.rcc.mux.usart234578sel = Usart234578sel::PCLK1;

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
