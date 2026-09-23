use core::fmt::{self, Display, Formatter};

/// Input or configuration error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EstimatorError {
    /// A value was NaN or infinite.
    NonFiniteInput,
    /// A time step, pressure, standard deviation, scale height, or gate was not positive.
    NonPositiveInput,
    /// A configured standard deviation was negative.
    NegativeStandardDeviation,
    /// A configuration value was outside its supported range.
    OutOfRangeInput,
    /// An IMU timestamp was not newer than the preceding timestamp for that IMU.
    NonMonotonicImuTimestamp,
    /// A measurement predates the retained filter history.
    MeasurementTooOld,
}

impl Display for EstimatorError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFiniteInput => formatter.write_str("estimator input is NaN or infinite"),
            Self::NonPositiveInput => formatter.write_str("estimator input must be positive"),
            Self::NegativeStandardDeviation => {
                formatter.write_str("estimator standard deviation must not be negative")
            }
            Self::OutOfRangeInput => formatter.write_str("estimator input is out of range"),
            Self::NonMonotonicImuTimestamp => {
                formatter.write_str("IMU timestamps must be strictly increasing")
            }
            Self::MeasurementTooOld => {
                formatter.write_str("measurement predates retained filter history")
            }
        }
    }
}

#[cfg(feature = "std")]
impl std::error::Error for EstimatorError {}

pub(crate) fn validate_finite(values: &[f32]) -> Result<(), EstimatorError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(EstimatorError::NonFiniteInput)
    }
}

pub(crate) fn validate_positive(values: &[f32]) -> Result<(), EstimatorError> {
    validate_finite(values)?;
    if values.iter().all(|value| *value > 0.0) {
        Ok(())
    } else {
        Err(EstimatorError::NonPositiveInput)
    }
}
