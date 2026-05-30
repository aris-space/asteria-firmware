pub fn clocks_config() -> embassy_stm32::Config {
    use embassy_stm32::rcc::{
        AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPreDiv, PllQDiv, PllRDiv,
        PllSource, Sysclk,
    };
    use embassy_stm32::rcc::mux;
    use embassy_stm32::time::Hertz;

    let mut config = embassy_stm32::Config::default();

    // clck running at 160 MHz (max for G473), pulsed
    // by 16 MHz external crystal
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

    // recommended when SYSCLK > 150 MHz on G4
    config.rcc.boost = true;

    // The STM32G4 reset value leaves the ADC12/ADC345 kernel clock disabled. Select SYSCLK here;
    // Embassy's G4 ADC driver then programs the ADC common prescaler from the selected kernel
    // clock, so 160 MHz SYSCLK becomes 40 MHz.
    config.rcc.mux.adc12sel = mux::Adcsel::SYS;
    config.rcc.mux.adc345sel = mux::Adcsel::SYS;

    config
}
