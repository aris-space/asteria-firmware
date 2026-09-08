use embassy_stm32::adc::AnyAdcChannel;

const SHUNT_RESISTANCE_OHM: f32 = 150.0;

pub(crate) const VOLTAGE_RANGE: [f32; 2] = [current_to_voltage(4.0), current_to_voltage(20.0)];
pub(crate) const UNDERFLOW_THRESHOLD_V: f32 = current_to_voltage((3.6 + 4.0) / 2.0);
pub(crate) const OVERFLOW_THRESHOLD_V: f32 = current_to_voltage((20.0 + 21.6) / 2.0);

const fn current_to_voltage(current_ma: f32) -> f32 {
    current_ma / 1000.0 * SHUNT_RESISTANCE_OHM
}

pub struct TrafagPSens<'a, ADC> {
    pub pin: AnyAdcChannel<'a, ADC>,
    pub si_range: [f32; 2],
}
