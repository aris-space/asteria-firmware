use asteria_sef_core::SelectorConfig;
use asteria_sef_light::{
    DualVerticalEstimator, EstimatorError, ImuAttitudeConfig, STANDARD_GRAVITY_MPS2,
    StationaryConfig, VerticalEstimatorSelectorConfig, VerticalFilterConfig,
};
use embassy_time::Duration;

use crate::sensors::IMU_COUNT;
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

// GNSS fixes reach the estimator up to ~145 ms after their time of validity.
pub const FUSION_DELAY_US: u64 = 200_000;
const PENDING_CAPACITY: usize = 512; // ~350 readings per fusion delay
pub const PUBLISH_PERIOD: Duration = Duration::from_millis(25);
pub const IMU_FRESH: Duration = Duration::from_millis(100);
pub const WARNING_PERIOD: Duration = Duration::from_secs(1);

// Accelerations are m/s² per sample. The IMU noise is far above the sensor's own,
// mostly for attitude errors, and the barometer's is 5x its measured 0.3 m to match.
const ACCELERATION_NOISE: f32 = 10.0;
const DEGRADED_ACCELERATION_NOISE: f32 = 20.0;
const SATURATED_ACCELERATION_NOISE: f32 = 1_000.0; // a clipped sample hides how much
const FREE_FALL_ACCELERATION_NOISE: f32 = 1.0; // falling, up is -g whatever the attitude
pub const BARO_HEIGHT_STD_M: f32 = 1.5;
const BARO_BIAS_WALK_M_PER_SQRT_S: f32 = 0.05;
const INITIAL_HEIGHT_STD_M: f32 = 1_000.0; // until GNSS references it to MSL
const INITIAL_VELOCITY_STD_MPS: f32 = 3.0;
const INITIAL_BARO_BIAS_STD_M: f32 = 200.0;
const INNOVATION_GATE_SIGMA: f32 = 5.0;

// GNSS errors last a minute or more, so 20 Hz epochs aren't independent: at rest
// the height spread 9 m against a reported 3 m, and walking it reported vertical
// speed the barometers didn't see.
pub const GNSS_HEIGHT_STD_SCALE: f32 = 10.0;
pub const GNSS_SPEED_STD_SCALE: f32 = 10.0;
pub const GNSS_MIN_STD: f32 = 0.1; // receivers can report zero

const AHRS_GAIN: f32 = 2.0;
const ACCEL_REJECTION_DEG: f32 = 10.0;
const ACCEL_RECOVERY_SAMPLES: u32 = 300;
const MAG_REJECTION_DEG: f32 = 20.0;
// One late or rejected sample shouldn't drop magnetometer aiding.
const MAG_MAX_AGE_US: u64 = mag::SAMPLE_INTERVAL.as_micros() * 5 / 2;
// The AHRS resets its attitude past this, so only when the gyro really clips.
const GYRO_RESET_DPS: f32 = 0.99 * GYRO_SATURATION_DPS;
// Calibration scales a clipped reading by a few percent either way.
const ACCEL_SATURATION_MPS2: f32 = 0.97 * ACCEL_RANGE_G * STANDARD_GRAVITY_MPS2;

// Only for estimators that model them, not SEF-light.
pub const LOCAL_MAGNETIC_FIELD_NED_NT: [f32; 3] = [26_375.0, -286.0, 35_701.0]; // WMMHR-2025, launch site, 2026-10-18
pub const MAG_DIRECTION_STD: f32 = 0.05;
pub const BARO_REFERENCE_PRESSURE_HPA: f32 = 1_013.25;
pub const BARO_PRESSURE_STD_HPA: f32 = 0.18; // BARO_HEIGHT_STD_M at ~0.12 hPa/m

// Barometers alone settle near 140 m (their bias prior); the first GNSS height
// brings it under 30 m.
pub const MSL_REFERENCED_HEIGHT_STD_M: f32 = 100.0;

pub type Estimator = DualVerticalEstimator<PENDING_CAPACITY>;

pub fn new_estimator() -> Result<Estimator, EstimatorError> {
    let filter = VerticalFilterConfig::new(
        ACCELERATION_NOISE,
        DEGRADED_ACCELERATION_NOISE,
        [BARO_BIAS_WALK_M_PER_SQRT_S; 2],
        INITIAL_HEIGHT_STD_M,
        INITIAL_VELOCITY_STD_MPS,
        [INITIAL_BARO_BIAS_STD_M; 2],
        INNOVATION_GATE_SIGMA,
    )?
    .with_saturated_acceleration_noise(SATURATED_ACCELERATION_NOISE)?
    .with_free_fall_acceleration_noise(FREE_FALL_ACCELERATION_NOISE)?;
    let attitude = ImuAttitudeConfig::new(
        AHRS_GAIN,
        GYRO_RESET_DPS,
        ACCEL_REJECTION_DEG,
        ACCEL_RECOVERY_SAMPLES,
    )?
    .with_magnetometer(MAG_REJECTION_DEG, MAG_MAX_AGE_US)?
    .with_accelerometer_saturation(ACCEL_SATURATION_MPS2)?;
    let selector = VerticalEstimatorSelectorConfig {
        score_memory: 0.95,
        maximum_nis_contribution: 25.0,
        degraded_score_penalty: 10.0,
        imu_selection: SelectorConfig {
            switch_hysteresis: 0.0025,
            minimum_dwell_time_us: 5_000_000,
            maximum_age_us: 100_000,
        },
        gnss_selection: SelectorConfig {
            minimum_dwell_time_us: 500_000,
            maximum_age_us: 250_000,
            ..SelectorConfig::default()
        },
    };
    let stationary = StationaryConfig {
        maximum_angular_rate_rad_s: 0.05,
        maximum_specific_force_error_mps2: 0.3,
        minimum_duration_us: 500_000,
        update_interval_us: 50_000,
        velocity_std_mps: 0.05,
        maximum_velocity_mps: 0.3,
        gnss_height_std_scale: 25.0,
    };
    DualVerticalEstimator::new(
        filter,
        [attitude; IMU_COUNT],
        selector,
        stationary,
        FUSION_DELAY_US,
    )
}
