use ads1120_thermocouples::thermocouple_conversions::ThermocoupleType;
use keller_pressure::sensor_def::{BasicPressure, DifferentialPressureLevel};

pub(crate) mod keller_p;
pub(crate) mod thermocouples;

pub(crate) const ACQ_PRESSURE_FREQUENCY_HZ: f32 = 60.0; // max stable possible ~ 60 Hz
pub(crate) const ACQ_THERMOCOUPLE_FREQ_HZ: f32 = 10.0;
pub(crate) const ACQ_TANK_LEVEL_FREQUENCY_HZ: f32 = 20.0;

pub(crate) const CAN_PRESSURE_FREQ_HZ: f32 = 20.0;
pub(crate) const CAN_TANK_LEVEL_FREQ_HZ: f32 = 20.0;
pub(crate) const CAN_TANK_TEMPERATURE_FREQ_HZ: f32 = 10.0;

pub(crate) const CAN_VALVE_STATES_FREQ_HZ: f32 = 5.0;
pub(crate) const CAN_BOARD_STATUS_FREQ_HZ: f32 = 1.0;

// Sensor Definitions
pub(crate) static OSS_TNK_P1: BasicPressure = BasicPressure { keller_id: 21 };

pub(crate) static OSS_TNK_P2: BasicPressure = BasicPressure { keller_id: 22 };

pub(crate) static OSS_TNK_LVL: DifferentialPressureLevel = DifferentialPressureLevel {
    keller_id: 23,
    zero_point: 0.0,
    full_point: 0.0750, // = 7500 Pa at LOx density of 1.141 g/cm3 for 100% tank level
};

pub(crate) static FSS_TNK_T: ThermocoupleType = ThermocoupleType::K;
