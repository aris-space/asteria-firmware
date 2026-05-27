// TODO: this is (mostly) a copy of the repo's shared/stm32h723_clocks.rs. Ideally the clock
// config (and the duplicated resources, see resources/mod.rs) move into a shared
// crate instead of being copied here.

pub fn clocks_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::mux::{I2c4sel, I2c1235sel, Saisel, Sdmmcsel, Usbsel};
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Hsi48Config, Pll, PllDiv, PllMul, PllPreDiv,
        PllSource, Sysclk, VoltageScale,
    };
    use embassy_stm32::time::Hertz;

    let mut config = embassy_stm32::Config::default();

    // 16 MHz XTAL
    config.rcc.hse = Some(Hse {
        freq: Hertz(16_000_000),
        mode: HseMode::Oscillator,
    });

    config.rcc.voltage_scale = VoltageScale::Scale0; // VOS0: top scale, needed for 544 MHz SYSCLK.

    // PLL1: VCO = 16 MHz * 34 = 544 MHz. Needs the cpu_freq_boost option byte set
    // (else embassy caps VOS0 at 520 and init() panics). 544 is the integer max under
    // the 550 MHz ceiling; exact 550 would need fractional-N.
    //   P = 544/1 = 544 MHz -> SYSCLK (CPU)
    //   Q = 544/4 = 136 MHz -> SPI123 kernel
    //   R = 544/2 = 272 MHz (not used explicitly)
    config.rcc.pll1 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL34,
        divp: Some(PllDiv::DIV1),
        divq: Some(PllDiv::DIV4),
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

    // Buses (respect H723 VOS0 limits: HCLK <= 275, PCLKx <= 137.5).
    config.rcc.d1c_pre = AHBPrescaler::DIV1; // CPU 544 MHz
    config.rcc.ahb_pre = AHBPrescaler::DIV2; // HCLK 272 MHz
    config.rcc.apb1_pre = APBPrescaler::DIV2; // 136 MHz
    config.rcc.apb2_pre = APBPrescaler::DIV2; // 136 MHz
    config.rcc.apb3_pre = APBPrescaler::DIV2; // 136 MHz
    config.rcc.apb4_pre = APBPrescaler::DIV2; // 136 MHz

    // Kernel muxes
    config.rcc.mux.spi123sel = Saisel::PLL1_Q; // 136 MHz for SPI1/2/3
    config.rcc.mux.i2c1235sel = I2c1235sel::PLL3_R; // 120 MHz for I2C1/2/3/5
    config.rcc.mux.i2c4sel = I2c4sel::PLL3_R; // 120 MHz for I2C4

    // SDMMC kernel: a dedicated 200 MHz PLL2_R (the SDMMC max) for the fastest card
    // clock. TODO: we'll need to implement this in the firmware too...
    config.rcc.pll2 = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL25, // 16 MHz * 25 = 400 MHz VCO
        divp: Some(PllDiv::DIV2),
        divq: Some(PllDiv::DIV2),
        divr: Some(PllDiv::DIV2), // 400 / 2 = 200 MHz -> SDMMC kernel
    });
    config.rcc.mux.sdmmcsel = Sdmmcsel::PLL2_R;

    // USB FS needs a 48 MHz kernel clock; HSI48 trimmed off USB SOF (CRS).
    config.rcc.hsi48 = Some(Hsi48Config {
        sync_from_usb: true,
    });
    config.rcc.mux.usbsel = Usbsel::HSI48;

    config
}
