use fusion_ahrs::{Ahrs, AhrsSettings, Convention};
use nalgebra::{UnitQuaternion, Vector3};

use crate::error::{EstimatorError, validate_finite, validate_positive};

/// Standard gravity used at the `fusion-ahrs` unit boundary.
pub const STANDARD_GRAVITY_MPS2: f32 = 9.806_65;

/// Calibrated accelerometer and gyroscope sample from one IMU.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuMeasurement {
    /// Specific force on the body axes in metres per second squared.
    ///
    /// A stationary, level forward-right-down sensor reads approximately
    /// `[0, 0, -STANDARD_GRAVITY_MPS2]`.
    pub acceleration_body_mps2: [f32; 3],
    /// Angular rate on the body axes in radians per second.
    pub angular_rate_body_rad_s: [f32; 3],
}

/// Configuration of one IMU attitude adapter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuAttitudeConfig {
    /// AHRS feedback gain. Zero disables accelerometer attitude correction after initialization.
    gain: f32,
    /// Gyroscope measurement range in degrees per second. Zero disables range recovery.
    gyroscope_range_deg_s: f32,
    /// Accelerometer rejection threshold from 0 to 90 degrees. Zero disables rejection.
    acceleration_rejection_deg: f32,
    /// Magnetometer rejection threshold in degrees. Zero disables rejection.
    magnetic_rejection_deg: f32,
    /// Oldest magnetometer sample still used for attitude, in microseconds. Zero disables
    /// magnetometer aiding.
    maximum_magnetometer_age_us: u64,
    /// Consecutive rejected samples before acceleration recovery.
    ///
    /// In `fusion-ahrs`, zero disables both acceleration rejection and recovery.
    recovery_trigger_period: u32,
}

impl ImuAttitudeConfig {
    /// Creates a validated IMU attitude configuration.
    ///
    /// # Errors
    ///
    /// Returns an error for non-finite values, negative gain or range, a rejection angle outside
    /// `0..=90` degrees, or a recovery period unsupported by `fusion-ahrs`.
    pub fn new(
        gain: f32,
        gyroscope_range_deg_s: f32,
        acceleration_rejection_deg: f32,
        recovery_trigger_period: u32,
    ) -> Result<Self, EstimatorError> {
        validate_finite(&[gain, gyroscope_range_deg_s, acceleration_rejection_deg])?;
        if gain < 0.0
            || gyroscope_range_deg_s < 0.0
            || !(0.0..=90.0).contains(&acceleration_rejection_deg)
            || recovery_trigger_period > i32::MAX as u32
        {
            return Err(EstimatorError::OutOfRangeInput);
        }

        Ok(Self {
            gain,
            gyroscope_range_deg_s,
            acceleration_rejection_deg,
            magnetic_rejection_deg: 0.0,
            maximum_magnetometer_age_us: 0,
            recovery_trigger_period,
        })
    }

    /// Enables calibrated magnetometer aiding.
    ///
    /// Each magnetometer sample is held and used for every IMU sample until a newer one arrives
    /// or it is older than `maximum_age_us`, after which attitude continues without it. If your
    /// magnetometer runs at a lower rate, you can raise `maximum_age_us` to a few of its sample
    /// periods. `rejection_deg` is the magnetic disturbance rejection threshold; zero disables
    /// rejection.
    ///
    /// # Errors
    ///
    /// Returns an error for a non-finite threshold, a threshold outside `0..=90` degrees, or a
    /// zero maximum age.
    pub fn with_magnetometer(
        mut self,
        rejection_deg: f32,
        maximum_age_us: u64,
    ) -> Result<Self, EstimatorError> {
        validate_finite(&[rejection_deg])?;
        if !(0.0..=90.0).contains(&rejection_deg) || maximum_age_us == 0 {
            return Err(EstimatorError::OutOfRangeInput);
        }
        self.magnetic_rejection_deg = rejection_deg;
        self.maximum_magnetometer_age_us = maximum_age_us;
        Ok(self)
    }
}

/// Compact AHRS operating flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ImuAttitudeFlags(u8);

impl ImuAttitudeFlags {
    const INITIALISING: u8 = 1 << 0;
    const ANGULAR_RATE_RECOVERY: u8 = 1 << 1;
    const ACCELERATION_RECOVERY: u8 = 1 << 2;

    /// Whether the AHRS is still using its initialization gain ramp.
    #[must_use]
    pub const fn initialising(self) -> bool {
        self.0 & Self::INITIALISING != 0
    }

    /// Whether the AHRS is recovering from a gyroscope range exceedance.
    #[must_use]
    pub const fn angular_rate_recovery(self) -> bool {
        self.0 & Self::ANGULAR_RATE_RECOVERY != 0
    }

    /// Whether persistent acceleration rejection triggered recovery.
    #[must_use]
    pub const fn acceleration_recovery(self) -> bool {
        self.0 & Self::ACCELERATION_RECOVERY != 0
    }
}

/// Diagnostics from one IMU attitude adapter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuAttitudeStatus {
    /// AHRS initialization and recovery flags.
    pub flags: ImuAttitudeFlags,
    /// Whether the accelerometer was excluded from the latest attitude correction.
    pub accelerometer_ignored: bool,
    /// Angular disagreement between measured and expected gravity in radians.
    pub acceleration_error_rad: f32,
    /// Whether the latest magnetic sample was excluded from attitude correction.
    pub magnetometer_ignored: bool,
    /// Angular disagreement between measured and expected magnetic field in radians.
    pub magnetic_error_rad: f32,
}

impl ImuAttitudeStatus {
    /// Whether vertical prediction should use the degraded acceleration-noise model.
    #[must_use]
    pub const fn acceleration_degraded(self) -> bool {
        self.flags.initialising()
            || self.flags.angular_rate_recovery()
            || self.flags.acceleration_recovery()
            || self.accelerometer_ignored
    }
}

/// Converts one IMU stream into up-positive, gravity-compensated vertical acceleration.
///
/// The adapter uses the project's north-east-down Earth frame. Inputs must already be calibrated.
pub struct ImuVerticalizer {
    ahrs: Ahrs,
    maximum_magnetometer_age_us: u64,
}

impl ImuVerticalizer {
    /// Creates an attitude adapter at the identity orientation.
    #[must_use]
    pub fn new(config: ImuAttitudeConfig) -> Self {
        Self {
            ahrs: Ahrs::with_settings(AhrsSettings {
                convention: Convention::Ned,
                gain: config.gain,
                gyroscope_range: config.gyroscope_range_deg_s,
                acceleration_rejection: config.acceleration_rejection_deg,
                magnetic_rejection: config.magnetic_rejection_deg,
                recovery_trigger_period: config.recovery_trigger_period,
            }),
            maximum_magnetometer_age_us: config.maximum_magnetometer_age_us,
        }
    }

    /// Returns the oldest magnetometer sample age still used for attitude, in microseconds.
    #[must_use]
    pub const fn maximum_magnetometer_age_us(&self) -> u64 {
        self.maximum_magnetometer_age_us
    }

    /// Sets the attitude from the gravity in one stationary sample, with zero heading. Without
    /// this the attitude starts level, and until it converges gravity leaks into the vertical
    /// acceleration of any board that is not level. A sample without measurable gravity leaves
    /// the attitude unchanged.
    pub fn align_to_gravity(&mut self, measurement: ImuMeasurement) {
        // At rest the accelerometer measures the reaction to gravity, so down is its negation.
        let Some(down) =
            (-Vector3::from(measurement.acceleration_body_mps2)).try_normalize(f32::EPSILON)
        else {
            return;
        };
        let orientation =
            UnitQuaternion::rotation_between(&down, &Vector3::z()).unwrap_or_else(|| {
                UnitQuaternion::from_axis_angle(&Vector3::x_axis(), core::f32::consts::PI)
            });
        self.ahrs.set_quaternion(orientation);
    }

    /// Updates attitude and returns up-positive, gravity-compensated acceleration in m/s².
    ///
    /// # Errors
    ///
    /// Returns an error for non-finite sensor values or a non-positive time step.
    pub fn update(
        &mut self,
        measurement: ImuMeasurement,
        dt_s: f32,
    ) -> Result<f32, EstimatorError> {
        self.update_with_magnetometer(measurement, None, dt_s)
    }

    /// Updates attitude with an optional calibrated body-frame magnetic field.
    /// The field may use any consistent unit because the AHRS normalizes it.
    pub fn update_with_magnetometer(
        &mut self,
        measurement: ImuMeasurement,
        magnetic_field_body: Option<[f32; 3]>,
        dt_s: f32,
    ) -> Result<f32, EstimatorError> {
        Self::validate_measurement(measurement)?;
        validate_positive(&[dt_s])?;

        let acceleration_g =
            Vector3::from(measurement.acceleration_body_mps2) / STANDARD_GRAVITY_MPS2;
        let angular_rate_dps =
            Vector3::from(measurement.angular_rate_body_rad_s) * (180.0 / core::f32::consts::PI);
        if let Some(field) = magnetic_field_body {
            validate_finite(&field)?;
            self.ahrs
                .update(angular_rate_dps, acceleration_g, Vector3::from(field), dt_s);
        } else {
            self.ahrs
                .update_no_magnetometer(angular_rate_dps, acceleration_g, dt_s);
        }
        Ok(-self.ahrs.earth_acceleration().z * STANDARD_GRAVITY_MPS2)
    }

    /// Returns the current body-to-NED orientation quaternion in `[w, x, y, z]` order.
    #[must_use]
    pub fn orientation_body_to_ned_wxyz(&self) -> [f32; 4] {
        let orientation = self.ahrs.quaternion();
        let quaternion = orientation.quaternion();
        [quaternion.w, quaternion.i, quaternion.j, quaternion.k]
    }

    /// Returns attitude-estimator diagnostics from the latest update.
    #[must_use]
    pub fn status(&self) -> ImuAttitudeStatus {
        let flags = self.ahrs.flags();
        let internal = self.ahrs.internal_states();
        let mut flag_bits = 0;
        if flags.initialising {
            flag_bits |= ImuAttitudeFlags::INITIALISING;
        }
        if flags.angular_rate_recovery {
            flag_bits |= ImuAttitudeFlags::ANGULAR_RATE_RECOVERY;
        }
        if flags.acceleration_recovery {
            flag_bits |= ImuAttitudeFlags::ACCELERATION_RECOVERY;
        }
        ImuAttitudeStatus {
            flags: ImuAttitudeFlags(flag_bits),
            accelerometer_ignored: internal.accelerometer_ignored,
            acceleration_error_rad: internal.acceleration_error.to_radians(),
            magnetometer_ignored: internal.magnetometer_ignored,
            magnetic_error_rad: internal.magnetic_error.to_radians(),
        }
    }

    /// Resets attitude to the identity orientation and restarts initialization.
    pub fn reset(&mut self) {
        self.ahrs.reset();
    }

    pub(crate) fn validate_measurement(measurement: ImuMeasurement) -> Result<(), EstimatorError> {
        validate_finite(&measurement.acceleration_body_mps2)?;
        validate_finite(&measurement.angular_rate_body_rad_s)
    }
}
