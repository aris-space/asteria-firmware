#![allow(dead_code)]
use ads1120_thermocouples::thermocouple_conversions::ThermocoupleType;

pub(crate) mod solenoid_current;
pub(crate) mod thermocouples;
pub(crate) mod trafag_p;

pub const CAN_BOARD_STATUS_FREQ_HZ: f32 = 1.0;
pub(crate) const ACQ_THERMOCOUPLE_FREQ_HZ: f32 = 10.0;
pub(crate) const ACQ_PRESSURE_FREQ_HZ: f32 = 500.0;
pub(crate) const ADC_CALIBRATION_SAMPLES: u64 = 50;

pub(crate) static OXD_TNK_T: ThermocoupleType = ThermocoupleType::K;
pub(crate) static OXD_RNL_T: ThermocoupleType = ThermocoupleType::K;

// Trafag P
pub(crate) const ENG_CC_P_RANGE: [f32; 2] = [0.0, 100.0];
pub(crate) const FUE_INJ_P_RANGE: [f32; 2] = [0.0, 100.0];
pub(crate) const OXD_INJ_P_RANGE: [f32; 2] = [0.0, 100.0];

#[derive(Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sensor {
    EngineP(f32),
}
