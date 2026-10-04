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

/// Keep roughly one independent GNSS height observation per second when a
/// receiver sends correlated fixes faster than that. A slower receiver keeps
/// its reported per-fix weight instead of receiving a fixed 20 Hz penalty.
pub fn correlated_gnss_height_std_m(height_std_m: f32, interval_us: u64) -> f32 {
    let interval_us = interval_us.clamp(50_000, 1_000_000);
    height_std_m * libm::sqrtf(1_000_000.0 / interval_us as f32)
}

/// When receivers disagree, retain that observed error as an uncertainty
/// floor for the less precise receiver if it later becomes the only fix.
/// Neither height is averaged or shifted.
pub fn weaker_gnss_disagreement_floor_m(
    heights_m: [f32; 2],
    velocity_down_mps: [f32; 2],
    timestamps_us: [u64; 2],
    stds_m: [f32; 2],
) -> [f32; 2] {
    // Bring both heights to the same epoch before comparing them in motion.
    let time_offset_s =
        (i128::from(timestamps_us[0]) - i128::from(timestamps_us[1])) as f32 / 1_000_000.0;
    let mean_velocity_down_mps = (velocity_down_mps[0] + velocity_down_mps[1]) * 0.5;
    let disagreement_m =
        (heights_m[0] - heights_m[1] + mean_velocity_down_mps * time_offset_s).abs();
    if stds_m[0] < stds_m[1] {
        [0.0, disagreement_m]
    } else if stds_m[1] < stds_m[0] {
        [disagreement_m, 0.0]
    } else {
        [disagreement_m; 2]
    }
}
// The weak four-satellite bench fix reported 2.7 m vAcc at PDOP 9.4.
// Use the good receiver's roughly 2.0 PDOP as the point where geometry
// starts raising the uncertainty floor.
const GNSS_PDOP_REFERENCE_CENTI: f32 = 200.0;
// Once a bias is fitted, keep its process uncertainty small so indoor GNSS
// wander does not move a steady barometric height estimate.
const STABLE_BARO_BIAS_WALK_M_PER_SQRT_S: f32 = 0.005;
const DRIFTING_BARO_BIAS_WALK_M_PER_SQRT_S: f32 = 0.5;
const BARO_TREND_WINDOW_US: u64 = 30_000_000;
const BARO_DRIFT_START_M: f32 = 0.5;
const BARO_DRIFT_STOP_M: f32 = 0.2;
// A stationary board showed >2 m of unexplained pressure-altitude change in
// five seconds; a 236 s quiet trace stayed below 1.2 m over five seconds.
const BARO_FAST_TREND_WINDOW_US: u64 = 5_000_000;
const BARO_FAST_DRIFT_START_M: f32 = 2.0;

pub fn new_estimator(gyroscope_range_deg_s: f32) -> Result<Estimator, EstimatorError> {
    // At 833 Hz, per-sample acceleration uncertainty below a few m/s² makes the filter
    // overconfident about velocity with the measured persistent 0.06 and 0.14 m/s² offsets.
    // Keep enough process uncertainty for barometric and GNSS updates to correct that drift.
    // GNSS establishes absolute MSL height. A standard-atmosphere conversion
    // gives each barometer an absolute observation, and the bias states absorb
    // local sea-level pressure and sensor calibration offsets.
    let filter = VerticalFilterConfig::new(
        10.0,                                    // healthy acceleration noise, m/s² per sample
        20.0,                                    // degraded acceleration noise, m/s² per sample
        [STABLE_BARO_BIAS_WALK_M_PER_SQRT_S; 2], // barometer-bias random walk, m/√s
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
    // Healthy chains have similar stationary scores. The board log showed a
    // 44-degree unaided heading difference at a marginal quality handover.
    // Require a clearer, sustained advantage before changing the full state.
    let selection = SelectorConfig::new(
        0.0025,    // IMU score improvement required for a handover
        5_000_000, // required improvement duration and minimum time between handovers, µs
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

/// Detects pressure-altitude change that GNSS vertical velocity cannot explain.
/// It changes only the bias process noise; SEF-light still estimates both biases.
#[derive(Default)]
pub struct BarometerBiasTracker {
    window_start: [Option<(u64, f32, f32)>; 2],
    drifting: [bool; 2],
}

impl BarometerBiasTracker {
    pub fn should_fit(&self, index: usize, bias_variance_m2: f32, barometer_std_m: f32) -> bool {
        self.drifting[index] || bias_variance_m2 > barometer_std_m * barometer_std_m
    }

    pub fn reset_windows(&mut self) {
        self.window_start = [None; 2];
    }

    pub fn observe(
        &mut self,
        index: usize,
        time_us: u64,
        pressure_altitude_m: f32,
        gnss_displacement_m: f32,
    ) -> Option<f32> {
        let Some((start_us, start_height_m, start_displacement_m)) = self.window_start[index]
        else {
            self.window_start[index] = Some((time_us, pressure_altitude_m, gnss_displacement_m));
            return None;
        };
        let elapsed_us = time_us.saturating_sub(start_us);
        if elapsed_us < BARO_FAST_TREND_WINDOW_US {
            return None;
        }
        let unexplained_change_m =
            (pressure_altitude_m - start_height_m) - (gnss_displacement_m - start_displacement_m);
        if !self.drifting[index] && unexplained_change_m.abs() > BARO_FAST_DRIFT_START_M {
            self.window_start[index] = Some((time_us, pressure_altitude_m, gnss_displacement_m));
            self.drifting[index] = true;
            return Some(DRIFTING_BARO_BIAS_WALK_M_PER_SQRT_S);
        }
        if elapsed_us < BARO_TREND_WINDOW_US {
            return None;
        }
        self.window_start[index] = Some((time_us, pressure_altitude_m, gnss_displacement_m));
        let should_drift = if self.drifting[index] {
            unexplained_change_m.abs() >= BARO_DRIFT_STOP_M
        } else {
            unexplained_change_m.abs() > BARO_DRIFT_START_M
        };
        if should_drift == self.drifting[index] {
            return None;
        }
        self.drifting[index] = should_drift;
        Some(if should_drift {
            DRIFTING_BARO_BIAS_WALK_M_PER_SQRT_S
        } else {
            STABLE_BARO_BIAS_WALK_M_PER_SQRT_S
        })
    }
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

pub fn gnss_height_std_m(vertical_accuracy_mm: u32, pdop_centi: u16) -> f32 {
    // vAcc already includes receiver geometry. Use PDOP as an additional
    // floor when vAcc looks too optimistic, rather than multiplying the two.
    (vertical_accuracy_mm as f32 / 1_000.0)
        .max(GNSS_HEIGHT_STD_FLOOR_M)
        .max(GNSS_HEIGHT_STD_FLOOR_M * pdop_centi as f32 / GNSS_PDOP_REFERENCE_CENTI)
}

pub fn gnss_measurement(input: GnssVerticalInput) -> GnssSample<VerticalGnssMeasurement> {
    // Consecutive GNSS heights share atmospheric and multipath errors. A receiver's
    // reported vertical accuracy can understate those slowly changing errors.
    let height_std_m = gnss_height_std_m(input.vertical_accuracy_mm, input.pdop_centi);
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
    fn barometer_bias_tracking_uses_change_unexplained_by_gnss_motion() {
        let mut tracker = BarometerBiasTracker::default();
        assert!(tracker.should_fit(0, 9.0, 1.5));
        assert!(!tracker.should_fit(0, 1.0, 1.5));
        assert_eq!(tracker.observe(0, 0, 300.0, 0.0), None);
        assert_eq!(tracker.observe(0, 5_000_000, 301.1, 0.0), None);
        assert_eq!(tracker.observe(0, 30_000_000, 302.0, 2.0), None);
        assert_eq!(
            tracker.observe(0, 60_000_000, 307.0, 2.0),
            Some(DRIFTING_BARO_BIAS_WALK_M_PER_SQRT_S)
        );
        assert_eq!(
            tracker.observe(0, 90_000_000, 309.0, 4.0),
            Some(STABLE_BARO_BIAS_WALK_M_PER_SQRT_S)
        );

        let mut fast_tracker = BarometerBiasTracker::default();
        assert_eq!(fast_tracker.observe(1, 0, 300.0, 0.0), None);
        assert_eq!(
            fast_tracker.observe(1, 5_000_000, 303.0, 0.0),
            Some(DRIFTING_BARO_BIAS_WALK_M_PER_SQRT_S)
        );
        assert!(fast_tracker.should_fit(1, 1.0, 1.5));
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
    fn weaker_gnss_geometry_increases_uncertainty_without_discarding_height() {
        assert_eq!(gnss_height_std_m(900, 180), 3.0);
        assert_eq!(gnss_height_std_m(4_000, 180), 4.0);
        assert!((gnss_height_std_m(2_700, 940) - 14.1).abs() < 1e-5);
    }

    #[test]
    fn gnss_height_weight_tracks_fix_interval_and_quality() {
        let one_hz_std = correlated_gnss_height_std_m(3.0, 1_000_000);
        let twenty_hz_std = correlated_gnss_height_std_m(3.0, 50_000);
        assert_eq!(one_hz_std, 3.0);
        assert!((20.0 / twenty_hz_std.powi(2) - 1.0 / one_hz_std.powi(2)).abs() < 1e-6);
        assert_eq!(correlated_gnss_height_std_m(6.0, 1_000_000), 6.0);
        assert!((correlated_gnss_height_std_m(6.0, 50_000) - 2.0 * twenty_hz_std).abs() < 1e-5);
    }

    #[test]
    fn observed_receiver_disagreement_raises_only_the_weaker_height_uncertainty() {
        assert_eq!(
            weaker_gnss_disagreement_floor_m([384.0, 415.0], [0.0; 2], [0; 2], [7.8, 4.8]),
            [31.0, 0.0]
        );
        assert_eq!(
            weaker_gnss_disagreement_floor_m([384.0, 415.0], [0.0; 2], [0; 2], [4.8, 7.8]),
            [0.0, 31.0]
        );
        assert_eq!(
            weaker_gnss_disagreement_floor_m([384.0, 415.0], [0.0; 2], [0; 2], [4.8, 4.8]),
            [31.0, 31.0]
        );
        assert_eq!(
            weaker_gnss_disagreement_floor_m(
                [105.0, 100.0],
                [-100.0; 2],
                [1_000_000, 950_000],
                [7.8, 4.8],
            ),
            [0.0, 0.0],
            "asynchronous fixes during ascent should agree after velocity alignment"
        );
    }

    #[test]
    fn weak_3d_height_update_is_accepted_with_less_weight() {
        fn corrected_height(vertical_accuracy_mm: u32, pdop_centi: u16) -> f32 {
            let mut estimator = new_estimator(2_000.0).unwrap();
            let mut anchor = gnss_measurement(GnssVerticalInput {
                height_msl_m: 420.0,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm: 900,
                speed_accuracy_mps: 0.1,
                fix_tier: 3,
                pdop_centi: 180,
            });
            anchor.measurement.height_std_m *= libm::sqrtf(20.0);
            estimator.update_gnss(0, [Some(anchor), None]).unwrap();
            let mut correction = gnss_measurement(GnssVerticalInput {
                height_msl_m: 440.0,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm,
                speed_accuracy_mps: 0.1,
                fix_tier: 3,
                pdop_centi,
            });
            correction.measurement.height_std_m *= libm::sqrtf(20.0);
            let updates = estimator
                .update_gnss(50_000, [Some(correction), None])
                .unwrap()
                .unwrap();
            assert!(updates[0].height.accepted);
            estimator.selected_state().height_m
        }

        let precise = corrected_height(900, 180);
        let weak = corrected_height(4_400, 550);
        assert!(precise > weak + 1.0, "precise={precise}, weak={weak}");
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
