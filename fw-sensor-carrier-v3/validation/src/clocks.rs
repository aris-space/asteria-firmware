// TODO: this is a copy of the repo's shared/stm32h723_clocks.rs. Ideally the clock
// config (and the duplicated resources, see resources/mod.rs) move into a shared
// crate instead of being copied here.

pub fn clocks_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::mux::{I2c4sel, I2c1235sel, Saisel, Usart16910sel, Usart234578sel};
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllDiv, PllMul, PllPreDiv, PllSource,
        Sysclk, VoltageScale,
    };
    use embassy_stm32::time::Hertz;

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
