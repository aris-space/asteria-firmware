//! Common sensor-update interface for estimator banks.

use crate::{BarometerInput, GnssInput, ImuInput, MagnetometerInput, NavigationState};

/// Failure while ingesting a sensor observation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UpdateError {
    InvalidSensor,
    InvalidMeasurement,
    NonMonotonicImuTimestamp,
    MeasurementTooOld,
    MeasurementInFuture,
    HistoryCapacityZero,
    AidingHistoryFull,
}

/// Capability to process IMU observations.
///
/// `Ok(())` means the input was processed. Detailed per-chain results remain available through
/// each bank's inherent methods.
pub trait ImuAided {
    /// Processes one IMU observation.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid source, sample, or timestamp.
    fn update_imu(&mut self, input: ImuInput) -> Result<(), UpdateError>;
}

/// Capability to process barometer observations.
pub trait BarometerAided {
    /// Processes one barometer observation.
    ///
    /// # Errors
    ///
    /// Returns an error if an estimator chain cannot process the observation.
    fn update_barometer(&mut self, input: BarometerInput) -> Result<(), UpdateError>;
}

/// Capability to process selected GNSS fixes.
pub trait GnssAided {
    /// Processes one receiver's selected GNSS fix.
    ///
    /// # Errors
    ///
    /// Returns an error if an estimator chain cannot process the selected observation.
    fn update_gnss(&mut self, input: GnssInput) -> Result<(), UpdateError>;
}

/// Capability to process magnetometer observations.
pub trait MagnetometerAided {
    /// Processes one calibrated magnetic-field observation.
    ///
    /// # Errors
    ///
    /// Returns an error for an invalid source or an observation that cannot be fused.
    fn update_magnetometer(&mut self, input: MagnetometerInput) -> Result<(), UpdateError>;
}

/// An estimator supporting all four sensor-update capabilities and reporting one navigation
/// solution.
///
/// `Ok(())` means the input was processed; innovation gating may still reject it. Detailed
/// per-chain fusion results remain available through each bank's inherent methods.
pub trait StateEstimator: ImuAided + BarometerAided + GnssAided + MagnetometerAided {
    //type Error; //TODO

    /// Returns the selected solution, propagated to the newest processed IMU sample.
    fn navigation_state(&self) -> NavigationState;
}
