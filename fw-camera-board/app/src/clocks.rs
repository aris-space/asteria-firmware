pub fn clocks_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::{AHBPrescaler, APBPrescaler, Hse, HseMode, Sysclk};
    use embassy_stm32::time::Hertz;

    let mut config = embassy_stm32::Config::default();

    // 16 MHz SYSCLK straight from the crystal to save power.
    config.rcc.hse = Some(Hse {
        freq: Hertz(16_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.sys = Sysclk::HSE;

    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;

    config
}
