pub fn clocks_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllQDiv, PllRDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::time::Hertz;

    let mut config = embassy_stm32::Config::default();

    // 160 MHz SYSCLK from the 16 MHz crystal.
    config.rcc.hse = Some(Hse {
        freq: Hertz(16_000_000),
        mode: HseMode::Oscillator,
    });
    config.rcc.pll = Some(Pll {
        source: PllSource::HSE,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL20,
        divr: Some(PllRDiv::DIV2),
        divq: Some(PllQDiv::DIV2),
        divp: None,
    });
    config.rcc.sys = Sysclk::PLL1_R;

    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;

    // Range 1 boost mode, needed above 150 MHz.
    config.rcc.boost = true;

    config
}
