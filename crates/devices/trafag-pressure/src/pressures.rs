use embassy_stm32::adc::AnyAdcChannel;

pub(crate) const VOLTAGE_RANGE: [f32; 2] = [0.4, 2.048]; // 4-20mA, 100Ω Resistor

pub struct TrafagPSens<'a, ADC> {
    pub pin: AnyAdcChannel<'a, ADC>,
    pub si_range: [f32; 2],
}
