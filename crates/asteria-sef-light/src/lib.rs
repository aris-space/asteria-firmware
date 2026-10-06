#![no_std]

//! Small vertical estimator for redundant rocket sensors.
//!
//! [`VerticalFilter`] is the four-state linear Kalman filter. [`DualVerticalEstimator`] adds one
//! independent attitude estimator and vertical filter per IMU. Both chains receive the same
//! barometer and GNSS observations, which makes their disagreement directly observable.

#[cfg(feature = "std")]
extern crate std;

mod dual;
mod error;
mod filter;
mod generated;
mod imu;
mod sensor_id;

pub use asteria_state_estimation::{
    DualGnssSelector, GNSS_RECEIVER_COUNT, GnssSample, GnssSelectorConfig, GnssSource,
    SelectedGnss, SelectorConfig,
};
pub use dual::{DualVerticalEstimator, VerticalDisagreement, VerticalEstimatorSelectorConfig};
pub use error::EstimatorError;
pub use filter::{
    MeasurementUpdate, PressureMeasurement, VerticalFilter, VerticalFilterConfig,
    VerticalGnssMeasurement, VerticalGnssUpdate, VerticalState, VerticalUncertainty,
};
pub use imu::{
    ImuAttitudeConfig, ImuAttitudeFlags, ImuAttitudeStatus, ImuMeasurement, ImuVerticalizer,
    STANDARD_GRAVITY_MPS2,
};
pub use sensor_id::{
    BARO_BUS_1, BARO_BUS_2, BAROMETER_COUNT, BarometerId, IMU_0, IMU_1, IMU_COUNT, ImuId,
};
