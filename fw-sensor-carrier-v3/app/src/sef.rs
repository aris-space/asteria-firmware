//! Unit and epoch conversion at the SEF-light input boundary.

use asteria_sef_light::{
    DualVerticalEstimator, EstimatorError, GnssSample, GnssSelectorConfig, ImuAttitudeConfig,
    ImuMeasurement, STANDARD_GRAVITY_MPS2, SelectorConfig, VerticalEstimatorSelectorConfig,
    VerticalFilterConfig, VerticalGnssMeasurement,
};

// Two 833 Hz IMUs produce about 667 events in 400 ms. The remaining capacity
// covers barometers, GNSS, and interrupt scheduling jitter.
pub const HISTORY_CAPACITY: usize = 768;
pub const MAX_AIDING_DELAY_US: u64 = 400_000;
pub type Estimator = DualVerticalEstimator<HISTORY_CAPACITY>;
pub const GNSS_HEIGHT_STD_FLOOR_M: f32 = 3.0;

pub fn new_estimator(gyroscope_range_deg_s: f32) -> Result<Estimator, EstimatorError> {
    // At 833 Hz, per-sample acceleration uncertainty below a few m/s² makes the filter
    // overconfident about velocity when an uncalibrated IMU has a persistent ~0.06 m/s² offset.
    // Keep enough process uncertainty for barometric and GNSS updates to correct that drift.
    // GNSS establishes absolute MSL height. A standard-atmosphere conversion
    // gives each barometer an absolute observation, and the bias states absorb
    // local sea-level pressure and sensor calibration offsets.
    let filter = VerticalFilterConfig::new(
        10.0,
        20.0,
        [0.001, 0.001],
        1_000.0,
        3.0,
        [200.0, 200.0],
        5.0,
    )?;
    let attitude = ImuAttitudeConfig::new(2.0, gyroscope_range_deg_s, 10.0, 300)?;
    let selection = SelectorConfig::new(2.0, 250_000).ok_or(EstimatorError::OutOfRangeInput)?;
    let selector = VerticalEstimatorSelectorConfig::new(0.95, 25.0, 10.0, 100_000, selection)?;
    let gnss = GnssSelectorConfig::new(3, 4.0, 500_000).ok_or(EstimatorError::OutOfRangeInput)?;
    Estimator::new(filter, [attitude; 2], selector, gnss, MAX_AIDING_DELAY_US)
}

pub fn barometric_pressure_altitude_m(pressure_mbar: f32) -> Option<f32> {
    if !pressure_mbar.is_finite() || pressure_mbar <= 0.0 {
        return None;
    }
    // ISA pressure altitude; the filter's barometer bias estimates the local
    // difference between this nominal conversion and GNSS MSL altitude.
    let height_m = 44_330.0 * (1.0 - libm::powf(pressure_mbar / 1_013.25, 0.190_294_95));
    height_m.is_finite().then_some(height_m)
}

pub fn imu_measurement(acceleration_g: [f32; 3], angular_rate_deg_s: [f32; 3]) -> ImuMeasurement {
    const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
    ImuMeasurement {
        acceleration_body_mps2: acceleration_g.map(|value| value * STANDARD_GRAVITY_MPS2),
        angular_rate_body_rad_s: angular_rate_deg_s.map(|value| value * DEG_TO_RAD),
    }
}

pub struct GnssVerticalInput {
    pub height_msl_m: f32,
    pub velocity_down_mps: f32,
    pub vertical_accuracy_mm: u32,
    pub speed_accuracy_mps: f32,
    pub fix_tier: u8,
    pub pdop_centi: u16,
}

pub fn gnss_measurement(input: GnssVerticalInput) -> GnssSample<VerticalGnssMeasurement> {
    // Consecutive GNSS heights share atmospheric and multipath errors. A receiver's
    // reported vertical accuracy can understate those slowly changing errors.
    let height_std_m = (input.vertical_accuracy_mm as f32 / 1000.0).max(GNSS_HEIGHT_STD_FLOOR_M);
    let velocity_std_mps = if input.speed_accuracy_mps > 0.0 {
        input.speed_accuracy_mps.max(0.1)
    } else {
        1.0
    };
    GnssSample {
        measurement: VerticalGnssMeasurement {
            height_m: input.height_msl_m,
            velocity_mps: -input.velocity_down_mps,
            height_std_m,
            velocity_std_mps,
        },
        fix_tier: input.fix_tier,
        pdop_centi: input.pdop_centi,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stationary_imu_and_gyroscope_units_match_sef_light() {
        let sample = imu_measurement([0.0, 0.0, -1.0], [180.0, 0.0, 0.0]);
        assert!((sample.acceleration_body_mps2[2] + STANDARD_GRAVITY_MPS2).abs() < 1e-5);
        assert!((sample.angular_rate_body_rad_s[0] - core::f32::consts::PI).abs() < 1e-5);
    }

    #[test]
    fn falling_pressure_produces_higher_barometric_altitude() {
        let at_900_mbar = barometric_pressure_altitude_m(900.0).unwrap();
        assert!(barometric_pressure_altitude_m(899.0).unwrap() > at_900_mbar + 9.0);
        assert_eq!(barometric_pressure_altitude_m(1_013.25), Some(0.0));
        assert_eq!(barometric_pressure_altitude_m(0.0), None);
    }

    #[test]
    fn gnss_uses_msl_height_and_up_positive_velocity() {
        let sample = gnss_measurement(GnssVerticalInput {
            height_msl_m: 1602.5,
            velocity_down_mps: -3.0,
            vertical_accuracy_mm: 2500,
            speed_accuracy_mps: 0.0,
            fix_tier: 3,
            pdop_centi: 120,
        });
        assert_eq!(sample.measurement.height_m, 1602.5);
        assert_eq!(sample.measurement.velocity_mps, 3.0);
        assert_eq!(sample.measurement.height_std_m, 3.0);
        assert_eq!(sample.measurement.velocity_std_mps, 1.0);
    }
}
