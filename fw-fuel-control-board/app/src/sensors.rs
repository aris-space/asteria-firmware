pub(crate) mod analog_p;

pub(crate) const ACQ_PRESSURE_FREQUENCY_HZ: f32 = 60.0; // max stable possible ~ 60 Hz

pub(crate) const CAN_PRESSURE_FREQ_HZ: f32 = 20.0;
pub(crate) const CAN_VALVE_STATES_FREQ_HZ: f32 = 5.0;
pub(crate) const CAN_BOARD_STATUS_FREQ_HZ: f32 = 1.0;
