use nalgebra::{SMatrix, SVector};

use crate::error::{EstimatorError, validate_finite, validate_positive};
use crate::generated;
use crate::sensor_id::{BARO_BUS_1, BARO_BUS_2, BAROMETER_COUNT, BarometerId};

mod idx {
    pub const ALT: usize = 0;
    pub const VEL: usize = 1;
    pub const BIAS_0: usize = 2;
    pub const BIAS_1: usize = 3;
    pub const SIZE: usize = 4;
}

type Vector4 = SVector<f32, { idx::SIZE }>;
type Matrix4 = SMatrix<f32, { idx::SIZE }, { idx::SIZE }>;

/// One calibrated barometric-height observation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PressureMeasurement {
    /// Up-positive height in metres.
    pub height_m: f32,
    /// Standard deviation assigned to the height observation in metres.
    pub height_std_m: f32,
}

/// Result of attempting to fuse one scalar measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MeasurementUpdate {
    /// Whether the measurement passed the innovation gate and was fused.
    pub accepted: bool,
    /// Measurement minus the value predicted from the state.
    pub innovation: f32,
    /// Squared innovation normalized by its predicted variance.
    pub normalized_innovation_squared: f32,
}

/// Vertical position and velocity observation from one GNSS solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerticalGnssMeasurement {
    /// Up-positive height in metres in the caller's absolute altitude frame.
    pub height_m: f32,
    /// Up-positive vertical velocity in m/s.
    pub velocity_mps: f32,
    /// Height standard deviation in metres.
    pub height_std_m: f32,
    /// Vertical-velocity standard deviation in m/s.
    pub velocity_std_mps: f32,
}

/// Result of fusing one vertical GNSS solution.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerticalGnssUpdate {
    /// Height correction result.
    pub height: MeasurementUpdate,
    /// Vertical-velocity correction result.
    pub velocity: MeasurementUpdate,
}

/// Estimated vertical state and sensor biases.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerticalState {
    /// Up-positive height in metres in the caller's absolute altitude frame.
    pub height_m: f32,
    /// Up-positive vertical velocity in m/s.
    pub velocity_mps: f32,
    /// Additive height biases for barometers zero and one in metres.
    pub barometer_bias_m: [f32; BAROMETER_COUNT],
}

impl VerticalState {
    fn from_vector(state: &Vector4) -> Self {
        Self {
            height_m: state[idx::ALT],
            velocity_mps: state[idx::VEL],
            barometer_bias_m: [state[idx::BIAS_0], state[idx::BIAS_1]],
        }
    }
}

/// Externally relevant uncertainty terms from the vertical covariance matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerticalUncertainty {
    /// Height variance in m².
    pub height_variance_m2: f32,
    /// Vertical-velocity variance in m²/s².
    pub velocity_variance_m2_per_s2: f32,
    /// Barometer height-bias variances in m², in barometer ID order.
    pub barometer_bias_variance_m2: [f32; BAROMETER_COUNT],
    /// Height and vertical-velocity covariance in m²/s.
    pub height_velocity_covariance_m2_per_s: f32,
}

/// Noise assumptions and initial uncertainty for [`VerticalFilter`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerticalFilterConfig {
    /// Per-sample standard deviation of vertical acceleration in m/s² while the AHRS is healthy.
    acceleration_noise_std_mps2: f32,
    /// Per-sample standard deviation used while the AHRS is initializing, recovering, or ignoring
    /// the accelerometer.
    degraded_acceleration_noise_std_mps2: f32,
    /// Barometer-bias random-walk standard deviations in m per square-root second.
    ///
    /// A value of zero stops adding process uncertainty to that bias. It can be changed at runtime
    /// with [`VerticalFilter::set_barometer_bias_walk_std`].
    barometer_bias_walk_std_m_per_sqrt_s: [f32; BAROMETER_COUNT],
    /// Initial height standard deviation in metres.
    initial_height_std_m: f32,
    /// Initial vertical-velocity standard deviation in m/s.
    initial_velocity_std_mps: f32,
    /// Initial standard deviation of each barometer height bias in metres.
    initial_barometer_bias_std_m: [f32; BAROMETER_COUNT],
    /// Scalar innovation gate in standard deviations.
    measurement_innovation_gate_sigma: f32,
}

impl VerticalFilterConfig {
    /// Creates validated vertical-filter noise and uncertainty settings.
    ///
    /// # Errors
    ///
    /// Returns an error for non-finite or negative standard deviations, degraded acceleration
    /// noise below the nominal value, or a non-positive innovation gate.
    pub fn new(
        acceleration_noise_std_mps2: f32,
        degraded_acceleration_noise_std_mps2: f32,
        barometer_bias_walk_std_m_per_sqrt_s: [f32; BAROMETER_COUNT],
        initial_height_std_m: f32,
        initial_velocity_std_mps: f32,
        initial_barometer_bias_std_m: [f32; BAROMETER_COUNT],
        measurement_innovation_gate_sigma: f32,
    ) -> Result<Self, EstimatorError> {
        let standard_deviations = [
            acceleration_noise_std_mps2,
            degraded_acceleration_noise_std_mps2,
            barometer_bias_walk_std_m_per_sqrt_s[0],
            barometer_bias_walk_std_m_per_sqrt_s[1],
            initial_height_std_m,
            initial_velocity_std_mps,
            initial_barometer_bias_std_m[0],
            initial_barometer_bias_std_m[1],
        ];
        validate_finite(&standard_deviations)?;
        if standard_deviations.iter().any(|value| *value < 0.0) {
            return Err(EstimatorError::NegativeStandardDeviation);
        }
        if degraded_acceleration_noise_std_mps2 < acceleration_noise_std_mps2 {
            return Err(EstimatorError::OutOfRangeInput);
        }
        validate_positive(&[measurement_innovation_gate_sigma])?;

        Ok(Self {
            acceleration_noise_std_mps2,
            degraded_acceleration_noise_std_mps2,
            barometer_bias_walk_std_m_per_sqrt_s,
            initial_height_std_m,
            initial_velocity_std_mps,
            initial_barometer_bias_std_m,
            measurement_innovation_gate_sigma,
        })
    }
}

/// Four-state linear Kalman filter for vertical motion.
///
/// The state is `[height, velocity, barometer 0 bias, barometer 1 bias]`. Height and velocity are
/// up-positive. A pressure observation from barometer `i` measures `height + barometer_bias[i]`.
/// GNSS measures height and velocity directly.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerticalFilter {
    state: Vector4,
    covariance: Matrix4,
    nominal_acceleration_noise_variance_mps4: f32,
    degraded_acceleration_noise_variance_mps4: f32,
    barometer_bias_walk_variance_m2_per_s: [f32; BAROMETER_COUNT],
    measurement_gate_squared: f32,
}

impl VerticalFilter {
    /// Creates a filter at zero height and zero vertical velocity.
    #[must_use]
    pub fn new(config: VerticalFilterConfig) -> Self {
        let mut covariance = Matrix4::zeros();
        covariance[(idx::ALT, idx::ALT)] = square(config.initial_height_std_m);
        covariance[(idx::VEL, idx::VEL)] = square(config.initial_velocity_std_mps);
        covariance[(idx::BIAS_0, idx::BIAS_0)] = square(config.initial_barometer_bias_std_m[0]);
        covariance[(idx::BIAS_1, idx::BIAS_1)] = square(config.initial_barometer_bias_std_m[1]);

        Self {
            state: Vector4::zeros(),
            covariance,
            nominal_acceleration_noise_variance_mps4: square(config.acceleration_noise_std_mps2),
            degraded_acceleration_noise_variance_mps4: square(
                config.degraded_acceleration_noise_std_mps2,
            ),
            barometer_bias_walk_variance_m2_per_s: [
                square(config.barometer_bias_walk_std_m_per_sqrt_s[0]),
                square(config.barometer_bias_walk_std_m_per_sqrt_s[1]),
            ],
            measurement_gate_squared: square(config.measurement_innovation_gate_sigma),
        }
    }

    /// Returns the current state estimate.
    #[must_use]
    pub fn state(&self) -> VerticalState {
        VerticalState::from_vector(&self.state)
    }

    /// Returns the externally relevant vertical uncertainty terms.
    #[must_use]
    pub fn uncertainty(&self) -> VerticalUncertainty {
        VerticalUncertainty {
            height_variance_m2: self.covariance[(idx::ALT, idx::ALT)],
            velocity_variance_m2_per_s2: self.covariance[(idx::VEL, idx::VEL)],
            barometer_bias_variance_m2: [
                self.covariance[(idx::BIAS_0, idx::BIAS_0)],
                self.covariance[(idx::BIAS_1, idx::BIAS_1)],
            ],
            height_velocity_covariance_m2_per_s: self.covariance[(idx::ALT, idx::VEL)],
        }
    }

    /// Changes how quickly one barometer bias may wander.
    ///
    /// Setting the standard deviation to zero stops adding random-walk uncertainty. Existing
    /// covariance remains, so later measurements can still correct the bias.
    ///
    /// # Errors
    ///
    /// Returns an error for a negative or non-finite standard deviation.
    pub fn set_barometer_bias_walk_std(
        &mut self,
        barometer: BarometerId,
        standard_deviation_m_per_sqrt_s: f32,
    ) -> Result<(), EstimatorError> {
        validate_finite(&[standard_deviation_m_per_sqrt_s])?;
        if standard_deviation_m_per_sqrt_s < 0.0 {
            return Err(EstimatorError::NegativeStandardDeviation);
        }
        self.barometer_bias_walk_variance_m2_per_s[barometer.index()] =
            square(standard_deviation_m_per_sqrt_s);
        Ok(())
    }

    /// Propagates the filter using up-positive, gravity-compensated vertical acceleration.
    ///
    /// # Errors
    ///
    /// Returns an error for non-finite acceleration or a non-positive time step.
    pub fn predict(&mut self, acceleration_up_mps2: f32, dt_s: f32) -> Result<(), EstimatorError> {
        let acceleration_noise_variance_mps4 = self.nominal_acceleration_noise_variance_mps4;
        self.predict_with_acceleration_noise(
            acceleration_up_mps2,
            dt_s,
            acceleration_noise_variance_mps4,
        )
    }

    pub(crate) fn predict_degraded(
        &mut self,
        acceleration_up_mps2: f32,
        dt_s: f32,
    ) -> Result<(), EstimatorError> {
        let acceleration_noise_variance_mps4 = self.degraded_acceleration_noise_variance_mps4;
        self.predict_with_acceleration_noise(
            acceleration_up_mps2,
            dt_s,
            acceleration_noise_variance_mps4,
        )
    }

    fn predict_with_acceleration_noise(
        &mut self,
        acceleration_up_mps2: f32,
        dt_s: f32,
        acceleration_noise_variance_mps4: f32,
    ) -> Result<(), EstimatorError> {
        validate_finite(&[acceleration_up_mps2])?;
        validate_positive(&[dt_s])?;

        let mut transition = Matrix4::zeros();
        let mut acceleration_jacobian = Vector4::zeros();
        self.state = generated::vertical_process_model(
            &self.state,
            acceleration_up_mps2,
            dt_s,
            Some(&mut transition),
            Some(&mut acceleration_jacobian),
        );
        let mut process_noise = acceleration_jacobian
            * acceleration_jacobian.transpose()
            * acceleration_noise_variance_mps4;
        process_noise[(idx::BIAS_0, idx::BIAS_0)] =
            self.barometer_bias_walk_variance_m2_per_s[0] * dt_s;
        process_noise[(idx::BIAS_1, idx::BIAS_1)] =
            self.barometer_bias_walk_variance_m2_per_s[1] * dt_s;

        self.covariance = transition * self.covariance * transition.transpose() + process_noise;
        symmetrize(&mut self.covariance);
        Ok(())
    }

    /// Fuses calibrated height from one barometer and estimates that barometer's height bias.
    ///
    /// # Errors
    ///
    /// Returns an error for non-finite height or a non-positive standard deviation.
    pub fn update_pressure(
        &mut self,
        barometer: BarometerId,
        measurement: PressureMeasurement,
    ) -> Result<MeasurementUpdate, EstimatorError> {
        validate_finite(&[measurement.height_m])?;
        validate_positive(&[measurement.height_std_m])?;

        let mut measurement_jacobian = Vector4::zeros();
        let predicted_height_m = match barometer {
            BARO_BUS_1 => {
                generated::barometer_0_model(&self.state, Some(&mut measurement_jacobian))[0]
            }
            BARO_BUS_2 => {
                generated::barometer_1_model(&self.state, Some(&mut measurement_jacobian))[0]
            }
            _ => unreachable!("unsupported barometer ID"),
        };
        Ok(self.update_scalar(
            measurement.height_m - predicted_height_m,
            &measurement_jacobian,
            square(measurement.height_std_m),
        ))
    }

    /// Fuses up-positive GNSS height and vertical velocity as two gated scalar observations.
    ///
    /// # Errors
    ///
    /// Returns an error for non-finite values or non-positive standard deviations.
    pub fn update_gnss(
        &mut self,
        measurement: VerticalGnssMeasurement,
    ) -> Result<VerticalGnssUpdate, EstimatorError> {
        validate_finite(&[measurement.height_m, measurement.velocity_mps])?;
        validate_positive(&[measurement.height_std_m, measurement.velocity_std_mps])?;

        let mut measurement_jacobian = SMatrix::zeros();
        let prediction = generated::gnss_model(&self.state, Some(&mut measurement_jacobian));
        let height_jacobian = measurement_jacobian.row(0).transpose();
        let height = self.update_scalar(
            measurement.height_m - prediction[0],
            &height_jacobian,
            square(measurement.height_std_m),
        );

        let prediction = generated::gnss_model(&self.state, None);
        let velocity_jacobian = measurement_jacobian.row(1).transpose();
        let velocity = self.update_scalar(
            measurement.velocity_mps - prediction[1],
            &velocity_jacobian,
            square(measurement.velocity_std_mps),
        );
        Ok(VerticalGnssUpdate { height, velocity })
    }

    fn update_scalar(
        &mut self,
        innovation: f32,
        measurement_jacobian: &Vector4,
        measurement_noise_variance: f32,
    ) -> MeasurementUpdate {
        let covariance_times_jacobian = self.covariance * measurement_jacobian;
        let innovation_variance =
            measurement_jacobian.dot(&covariance_times_jacobian) + measurement_noise_variance;
        let normalized_innovation_squared = square(innovation) / innovation_variance;
        let accepted = normalized_innovation_squared <= self.measurement_gate_squared;
        if accepted {
            let kalman_gain = covariance_times_jacobian / innovation_variance;
            self.state += kalman_gain * innovation;
            let joseph_correction =
                Matrix4::identity() - kalman_gain * measurement_jacobian.transpose();
            self.covariance = joseph_correction * self.covariance * joseph_correction.transpose()
                + (kalman_gain * kalman_gain.transpose()) * measurement_noise_variance;
            symmetrize(&mut self.covariance);
        }
        MeasurementUpdate {
            accepted,
            innovation,
            normalized_innovation_squared,
        }
    }
}

const fn square(value: f32) -> f32 {
    value * value
}

fn symmetrize(matrix: &mut Matrix4) {
    for row in 0..idx::SIZE {
        for column in 0..row {
            let value = f32::midpoint(matrix[(row, column)], matrix[(column, row)]);
            matrix[(row, column)] = value;
            matrix[(column, row)] = value;
        }
    }
}
