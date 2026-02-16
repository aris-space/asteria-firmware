#![allow(dead_code)]
use ads1120_thermocouples::thermocouple_conversions::ThermocoupleType;
use keller_pressure::sensor_def::BasicPressure;

pub(crate) mod keller_p;
pub(crate) mod thermocouples;
pub(crate) mod trafag_p;

pub const CAN_PRESSURE_FREQ_HZ: f32 = 20.0;
pub const CAN_THERMOCOUPLE_FREQ_HZ: f32 = 10.0;
pub(crate) const ACQ_THERMOCOUPLE_FREQ_HZ: f32 = 10.0;
pub(crate) const ACQ_PRESSURE_FREQ_HZ: f32 = 50.0;
pub(crate) const ADC_CALIBRATION_SAMPLES: u64 = 50;

pub(crate) static OXD_TNK_T: ThermocoupleType = ThermocoupleType::K;
pub(crate) static OXD_RNL_T: ThermocoupleType = ThermocoupleType::K;

// Trafag P
pub(crate) const ENG_CC_P_RANGE: [f32; 2] = [0.0, 250.0];
pub(crate) const FUE_INJ_P_RANGE: [f32; 2] = [0.0, 250.0];
pub(crate) const OXD_INJ_P_RANGE: [f32; 2] = [0.0, 250.0];

// Keller P
pub(crate) static ENG_CC_P: BasicPressure = BasicPressure { keller_id: 31 };
pub(crate) static FUE_INJ_P: BasicPressure = BasicPressure { keller_id: 30 };
pub(crate) static OXD_INJ_P: BasicPressure = BasicPressure { keller_id: 32 };

// Only the EngineP and IgniterP are implemented for the detections
#[derive(Clone, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Sensor {
    EngineP(f32),
    IgniterP(f32),
}
