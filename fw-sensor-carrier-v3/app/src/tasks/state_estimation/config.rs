use asteria_sef_light::{
    DualVerticalEstimator, EstimatorConfig, EstimatorError, ImuAttitudeConfig,
    STANDARD_GRAVITY_MPS2, SelectionConfig, SelectorConfig, StationaryConfig, VerticalFilterConfig,
};
use embassy_time::Duration;

use crate::tasks::readout::{
    imu::{ACCEL_RANGE_G, GYRO_SATURATION_DPS},
    mag,
};

// Only has to be within a few hundred metres of the flight.
pub const LAUNCH_SITE_LATITUDE_DEG: f64 = 39.389_231_529_748_535;
pub const LAUNCH_SITE_LONGITUDE_DEG: f64 = -8.287_750_809_600_846;

// Taken from https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm
// Calculated for the launch site near Constância, Portugal
// Model Used:  WMMHR-2025
// Latitude:    39° 23' 21" N
// Longitude:   8° 17' 16" W
// Elevation:   0.0 m Mean Sea Level
//
// 2026-10-18   Declination: 0° 37' 19" W
//              changing by +0° 9' 9"/yr
//              uncertainty ±0° 19' (1σ)
pub const MAGNETIC_DECLINATION_DEG: f32 = -(37.0 / 60.0 + 19.0 / 3600.0);
// Reference field and noise for estimators that model the magnetic field.
pub const LOCAL_MAGNETIC_FIELD_NED_NT: [f32; 3] = [26_375.0, -286.0, 35_701.0];
pub const MAG_DIRECTION_STD: f32 = 0.05;

// GNSS fixes reach the estimator up to ~145 ms after their time of validity.
pub const FUSION_DELAY_US: u64 = 200_000;
const PENDING_CAPACITY: usize = 512; // ~350 readings per fusion delay

pub const PUBLISH_PERIOD: Duration = Duration::from_millis(25);
pub const IMU_FRESH: Duration = Duration::from_millis(100);
pub const WARNING_PERIOD: Duration = Duration::from_secs(1);

// Acceleration noise (m/s²) is inflated to account for attitude errors.
const ACCELERATION_NOISE: f32 = 10.0;
const DEGRADED_ACCELERATION_NOISE: f32 = 20.0;
// Clipping hides the acceleration magnitude.
const SATURATED_ACCELERATION_NOISE: f32 = 1_000.0;
// Free-fall acceleration is known even when attitude is uncertain.
const FREE_FALL_ACCELERATION_NOISE: f32 = 1.0;

pub const BARO_HEIGHT_STD_M: f32 = 1.5; // 5x the measured 0.3 m noise
const BARO_BIAS_WALK_M_PER_SQRT_S: f32 = 0.05;
// Pressure inputs support estimators that model pressure directly.
pub const BARO_REFERENCE_PRESSURE_HPA: f32 = 1_013.25;
pub const BARO_PRESSURE_STD_HPA: f32 = 0.18; // BARO_HEIGHT_STD_M at ~0.12 hPa/m

const INITIAL_HEIGHT_STD_M: f32 = 1_000.0; // until GNSS references it to MSL
const INITIAL_VELOCITY_STD_MPS: f32 = 3.0;
const INITIAL_BARO_BIAS_STD_M: f32 = 200.0;
const INNOVATION_GATE_SIGMA: f32 = 5.0;

// GNSS errors persist across fixes. Inflate vertical uncertainties to match
// the observed height spread and unreliable vertical speed.
pub const GNSS_HEIGHT_STD_SCALE: f32 = 10.0;
pub const GNSS_SPEED_STD_SCALE: f32 = 10.0;
pub const GNSS_MIN_STD: f32 = 0.1; // receivers can report zero

const AHRS_GAIN: f32 = 2.0;
const ACCEL_REJECTION_DEG: f32 = 10.0;
const ACCEL_RECOVERY_SAMPLES: u32 = 300;
const MAG_REJECTION_DEG: f32 = 20.0;
// Tolerate one late or rejected magnetometer sample.
const MAG_MAX_AGE_US: u64 = mag::SAMPLE_INTERVAL.as_micros() * 5 / 2;

// Reserve AHRS resets for gyro clipping.
const GYRO_RESET_DPS: f32 = 0.99 * GYRO_SATURATION_DPS;
// Allow for calibration scaling near accelerometer saturation.
const ACCEL_SATURATION_MPS2: f32 = 0.97 * ACCEL_RANGE_G * STANDARD_GRAVITY_MPS2;

// Observed height uncertainty: ~140 m with barometers alone, <30 m with GNSS.
pub const MSL_REFERENCED_HEIGHT_STD_M: f32 = 100.0;

pub type Estimator = DualVerticalEstimator<PENDING_CAPACITY>;

pub fn new_estimator() -> Result<Estimator, EstimatorError> {
    Estimator::new(EstimatorConfig {
        filter: VerticalFilterConfig {
            acceleration_noise_std_mps2: ACCELERATION_NOISE,
            degraded_acceleration_noise_std_mps2: DEGRADED_ACCELERATION_NOISE,
            saturated_acceleration_noise_std_mps2: SATURATED_ACCELERATION_NOISE,
            free_fall_acceleration_noise_std_mps2: FREE_FALL_ACCELERATION_NOISE,
            barometer_bias_walk_std_m_per_sqrt_s: BARO_BIAS_WALK_M_PER_SQRT_S,
            initial_height_std_m: INITIAL_HEIGHT_STD_M,
            initial_velocity_std_mps: INITIAL_VELOCITY_STD_MPS,
            initial_barometer_bias_std_m: INITIAL_BARO_BIAS_STD_M,
            innovation_gate_sigma: INNOVATION_GATE_SIGMA,
        },
        attitude: ImuAttitudeConfig {
            gain: AHRS_GAIN,
            gyroscope_range_deg_s: GYRO_RESET_DPS,
            acceleration_rejection_deg: ACCEL_REJECTION_DEG,
            recovery_trigger_period: ACCEL_RECOVERY_SAMPLES,
            magnetic_rejection_deg: MAG_REJECTION_DEG,
            maximum_magnetometer_age_us: MAG_MAX_AGE_US,
            accelerometer_saturation_mps2: ACCEL_SATURATION_MPS2,
        },
        selection: SelectionConfig {
            score_memory: 0.95,
            maximum_nis_contribution: 25.0,
            degraded_score_penalty: 10.0,
            imu: SelectorConfig {
                switch_hysteresis: 0.0025,
                minimum_dwell_time_us: 5_000_000,
                maximum_age_us: 100_000,
            },
            gnss: SelectorConfig {
                switch_hysteresis: 0.0,
                minimum_dwell_time_us: 500_000,
                maximum_age_us: 250_000,
            },
        },
        stationary: StationaryConfig {
            maximum_angular_rate_rad_s: 0.05,
            maximum_specific_force_error_mps2: 0.3,
            minimum_duration_us: 500_000,
            update_interval_us: 50_000,
            velocity_std_mps: 0.05,
            maximum_velocity_mps: 0.3,
            gnss_height_std_scale: 25.0,
        },
        fusion_delay_us: FUSION_DELAY_US,
    })
}
