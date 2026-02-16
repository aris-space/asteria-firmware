use keller_pressure::sensor_def::BasicPressure;

pub(crate) mod keller_p;

pub(crate) const ACQ_PRESSURE_FREQUENCY_HZ: f32 = 60.0; // max stable possible ~ 60 Hz

pub(crate) const CAN_PRESSURE_FREQ_HZ: f32 = 20.0;
pub(crate) const CAN_VALVE_STATES_FREQ_HZ: f32 = 5.0;
pub(crate) const CAN_BOARD_STATUS_FREQ_HZ: f32 = 1.0;

// Sensor Definitions
pub(crate) static PRZ_MNL_P: BasicPressure = BasicPressure { keller_id: 10 };

pub(crate) static FSS_TNK_P1: BasicPressure = BasicPressure { keller_id: 11 };

pub(crate) static FSS_TNK_P2: BasicPressure = BasicPressure { keller_id: 12 };
