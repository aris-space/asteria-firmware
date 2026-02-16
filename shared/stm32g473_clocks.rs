pub fn clocks_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::{
        Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv, PllRDiv, PllSource,
    };
    use embassy_stm32::time::Hertz;

    let mut config = embassy_stm32::Config::default();

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

    config
}
