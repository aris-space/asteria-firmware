pub(crate) mod keller_analog_p;
pub(crate) mod solenoid_current;

pub(crate) const ACQ_PRESSURE_FREQ_HZ: f32 = 60.0; // max stable possible ~ 60 Hz
pub(crate) const ADC_CALIBRATION_SAMPLES: u64 = 50;

pub(crate) const OXIDIZER_TANK_PRESSURE_1_RANGE: [f32; 2] = [0.0, 100.0];
pub(crate) const OXIDIZER_TANK_PRESSURE_2_RANGE: [f32; 2] = [0.0, 200.0];
pub(crate) const OXIDIZER_TANK_DIFFERENTIAL_PRESSURE_RANGE: [f32; 2] = [0.0, 200.0];

pub(crate) const CAN_BOARD_STATUS_FREQ_HZ: f32 = 1.0;
