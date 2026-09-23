//! Unit and epoch conversion at the SEF-light input boundary.

use asteria_sef_light::{
    DualVerticalEstimator, EstimatorError, GnssSample, GnssSelectorConfig, ImuAttitudeConfig,
    ImuId, ImuMeasurement, STANDARD_GRAVITY_MPS2, SelectorConfig, VerticalEstimatorSelectorConfig,
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
    // overconfident about velocity with the measured persistent 0.06 and 0.14 m/s² offsets.
    // Keep enough process uncertainty for barometric and GNSS updates to correct that drift.
    // GNSS establishes absolute MSL height. A standard-atmosphere conversion
    // gives each barometer an absolute observation, and the bias states absorb
    // local sea-level pressure and sensor calibration offsets.
    let filter = VerticalFilterConfig::new(
        10.0,           // healthy acceleration noise, m/s² per sample
        20.0,           // degraded acceleration noise, m/s² per sample
        [0.001, 0.001], // barometer-bias random walk, m/√s
        1_000.0,        // initial height uncertainty, m; GNSS establishes MSL
        3.0,            // initial vertical-velocity uncertainty, m/s
        [200.0, 200.0], // initial pressure-altitude bias uncertainty, m
        5.0,            // measurement innovation gate, standard deviations
    )?;
    let attitude = ImuAttitudeConfig::new(
        2.0, // AHRS feedback gain
        gyroscope_range_deg_s,
        10.0, // accelerometer rejection angle, degrees
        300,  // rejected samples before acceleration recovery
    )?
    .with_magnetic_rejection(20.0)?;
    // Healthy chains have similar stationary scores. Require a sustained
    // improvement before switching to avoid brief attitude discontinuities.
    let selection = SelectorConfig::new(
        0.001,     // IMU score improvement required for a handover
        1_500_000, // required improvement duration and minimum time between handovers, µs
    )
    .ok_or(EstimatorError::OutOfRangeInput)?;
    let selector = VerticalEstimatorSelectorConfig::new(
        0.95,    // previous score weight
        25.0,    // maximum contribution from one innovation
        10.0,    // degraded acceleration penalty
        100_000, // maximum IMU sample age, µs
        selection,
    )?;
    let gnss = GnssSelectorConfig::new(
        3,       // minimum fix tier
        4.0,     // inter-receiver consistency gate, standard deviations
        500_000, // minimum time between receiver handovers, µs
    )
    .ok_or(EstimatorError::OutOfRangeInput)?;
    Estimator::new(filter, [attitude; 2], selector, gnss, MAX_AIDING_DELAY_US)
}

/// Keep a live attitude chain through vertical quality handovers. Its heading
/// is otherwise unrelated to the other chain until magnetic aiding is calibrated.
pub fn select_orientation_imu(
    current: Option<ImuId>,
    selected_vertical: ImuId,
    current_is_fresh: bool,
) -> ImuId {
    current
        .filter(|_| current_is_fresh)
        .unwrap_or(selected_vertical)
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

/// Short-window IMU diagnostics. Mean angular rate estimates gyro bias only
/// while the carrier is known to be stationary.
#[derive(Clone, Copy, Default)]
pub struct ImuWindow {
    samples: u32,
    gravity_error_sum: f32,
    gravity_error_square_sum: f32,
    max_acceleration_mps2: f32,
    gyro_sum: [f32; 3],
    gyro_square_sum: f32,
    max_gyro_rad_s: f32,
}

#[derive(Clone, Copy)]
pub struct ImuWindowSummary {
    pub samples: u32,
    pub gravity_error_mean_mps2: f32,
    pub gravity_error_noise_mps2: f32,
    pub max_acceleration_mps2: f32,
    pub gyro_mean_rad_s: [f32; 3],
    pub gyro_noise_rad_s: f32,
    pub max_gyro_rad_s: f32,
}

impl ImuWindow {
    pub fn samples(&self) -> u32 {
        self.samples
    }

    pub fn record(&mut self, measurement: ImuMeasurement) {
        let acceleration_square = measurement
            .acceleration_body_mps2
            .iter()
            .map(|value| value * value)
            .sum::<f32>();
        let acceleration = libm::sqrtf(acceleration_square);
        let gravity_error = acceleration - STANDARD_GRAVITY_MPS2;
        self.samples += 1;
        self.gravity_error_sum += gravity_error;
        self.gravity_error_square_sum += gravity_error * gravity_error;
        self.max_acceleration_mps2 = self.max_acceleration_mps2.max(acceleration);
        let mut gyro_square = 0.0;
        for (sum, rate) in self
            .gyro_sum
            .iter_mut()
            .zip(measurement.angular_rate_body_rad_s)
        {
            *sum += rate;
            self.gyro_square_sum += rate * rate;
            gyro_square += rate * rate;
        }
        self.max_gyro_rad_s = self.max_gyro_rad_s.max(libm::sqrtf(gyro_square));
    }

    pub fn summary(self) -> Option<ImuWindowSummary> {
        if self.samples == 0 {
            return None;
        }
        let count = self.samples as f32;
        let gravity_error_mean = self.gravity_error_sum / count;
        let gyro_mean = self.gyro_sum.map(|sum| sum / count);
        let gyro_mean_square = gyro_mean.iter().map(|rate| rate * rate).sum::<f32>();
        Some(ImuWindowSummary {
            samples: self.samples,
            gravity_error_mean_mps2: gravity_error_mean,
            gravity_error_noise_mps2: libm::sqrtf(
                (self.gravity_error_square_sum / count - gravity_error_mean * gravity_error_mean)
                    .max(0.0),
            ),
            max_acceleration_mps2: self.max_acceleration_mps2,
            gyro_mean_rad_s: gyro_mean,
            gyro_noise_rad_s: libm::sqrtf(
                (self.gyro_square_sum / count - gyro_mean_square).max(0.0),
            ),
            max_gyro_rad_s: self.max_gyro_rad_s,
        })
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

    #[test]
    fn imu_window_separates_stationary_offset_from_noise() {
        let mut window = ImuWindow::default();
        for index in 0_u32..100 {
            let perturbation = if index.is_multiple_of(2) { 0.02 } else { -0.02 };
            window.record(ImuMeasurement {
                acceleration_body_mps2: [0.0, 0.0, -STANDARD_GRAVITY_MPS2 - 0.2 - perturbation],
                angular_rate_body_rad_s: [0.001, 0.0, 0.0],
            });
        }
        let summary = window.summary().unwrap();
        assert_eq!(summary.samples, 100);
        assert!((summary.gravity_error_mean_mps2 - 0.2).abs() < 1e-4);
        assert!((summary.gravity_error_noise_mps2 - 0.02).abs() < 1e-3);
    }
}
