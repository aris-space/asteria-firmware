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

// Let the bias follow gradual pressure drift while GNSS keeps MSL anchored.
const BARO_BIAS_WALK_M_PER_SQRT_S: f32 = 0.02;

pub fn new_estimator(gyroscope_range_deg_s: f32) -> Result<Estimator, EstimatorError> {
    // At 833 Hz, per-sample acceleration uncertainty below a few m/s² makes the filter
    // overconfident about velocity with the measured persistent 0.06 and 0.14 m/s² offsets.
    // Keep enough process uncertainty for barometric and GNSS updates to correct that drift.
    // GNSS establishes absolute MSL height. A standard-atmosphere conversion
    // gives each barometer an absolute observation, and the bias states absorb
    // local sea-level pressure and sensor calibration offsets.
    let filter = VerticalFilterConfig::new(
        10.0,                             // healthy acceleration noise, m/s² per sample
        20.0,                             // degraded acceleration noise, m/s² per sample
        [BARO_BIAS_WALK_M_PER_SQRT_S; 2], // barometer-bias random walk, m/√s
        1_000.0,                          // initial height uncertainty, m; GNSS establishes MSL
        3.0,                              // initial vertical-velocity uncertainty, m/s
        [200.0, 200.0],                   // initial pressure-altitude bias uncertainty, m
        5.0,                              // measurement innovation gate, standard deviations
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

/// Converts a receiver solution to SEF-light's up-positive vertical
/// measurement, using the receiver's reported accuracies as standard deviations.
pub fn gnss_measurement(input: GnssVerticalInput) -> GnssSample<VerticalGnssMeasurement> {
    const MIN_STD: f32 = 0.1;
    GnssSample {
        measurement: VerticalGnssMeasurement {
            height_m: input.height_msl_m,
            velocity_mps: -input.velocity_down_mps,
            height_std_m: (input.vertical_accuracy_mm as f32 / 1_000.0).max(MIN_STD),
            velocity_std_mps: input.speed_accuracy_mps.max(MIN_STD),
        },
        fix_tier: input.fix_tier,
        pdop_centi: input.pdop_centi,
    }
}
