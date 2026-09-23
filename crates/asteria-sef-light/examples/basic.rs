use asteria_sef_light::{
    BARO_BUS_1, BARO_BUS_2, DualVerticalEstimator, EstimatorError, GnssSample, GnssSelectorConfig,
    IMU_0, IMU_1, ImuAttitudeConfig, ImuMeasurement, PressureMeasurement, SelectorConfig,
    VerticalEstimatorSelectorConfig, VerticalFilterConfig, VerticalGnssMeasurement,
};

fn main() -> Result<(), EstimatorError> {
    let filter_config =
        VerticalFilterConfig::new(0.5, 5.0, [0.02, 0.02], 10.0, 3.0, [5.0, 5.0], 5.0)?;
    let attitude_config = ImuAttitudeConfig::new(0.5, 2_000.0, 20.0, 500)?;
    let selection = SelectorConfig::new(2.0, 250_000).ok_or(EstimatorError::OutOfRangeInput)?;
    let selector_config =
        VerticalEstimatorSelectorConfig::new(0.95, 25.0, 10.0, 100_000, selection)?;
    let gnss_selector_config =
        GnssSelectorConfig::new(3, 4.0, 500_000).ok_or(EstimatorError::OutOfRangeInput)?;
    // Size history for the combined sensor event rate and measured aiding delay. At two 833 Hz
    // IMUs and two 40 Hz barometers, 256 entries cover roughly 120 ms with some margin.
    let mut estimator = DualVerticalEstimator::<256>::new(
        filter_config,
        [attitude_config, attitude_config],
        selector_config,
        gnss_selector_config,
        120_000,
    )?;

    // Call this independently with each IMU's monotonic capture timestamp. The first sample primes
    // that IMU's clock; later samples update its chain. A stationary, level FRD IMU reads
    // approximately [0, 0, -9.80665] m/s².
    let stationary_imu = ImuMeasurement {
        acceleration_body_mps2: [0.0, 0.0, -asteria_sef_light::STANDARD_GRAVITY_MPS2],
        angular_rate_body_rad_s: [0.0, 0.0, 0.0],
    };
    let imu_0_priming_advanced = estimator.update_imu(IMU_0, 1_000_000, stationary_imu)?;
    let imu_1_priming_advanced = estimator.update_imu(IMU_1, 1_000_100, stationary_imu)?;
    let imu_0_advanced = estimator.update_imu(IMU_0, 1_005_000, stationary_imu)?;
    let imu_1_advanced = estimator.update_imu(IMU_1, 1_005_100, stationary_imu)?;

    // Every barometer observation is applied to both IMU/filter chains.
    let barometer_0_updates = estimator.update_pressure(
        1_005_000,
        BARO_BUS_1,
        PressureMeasurement {
            height_m: 1.7,
            height_std_m: 2.55,
        },
    )?;

    let barometer_1_updates = estimator.update_pressure(
        1_005_000,
        BARO_BUS_2,
        PressureMeasurement {
            height_m: 1.7,
            height_std_m: 2.55,
        },
    )?;

    // Submit one compensated GNSS epoch containing whichever receiver solutions are available.
    let gnss_measurement = VerticalGnssMeasurement {
        height_m: 2.1,
        velocity_mps: 0.2,
        height_std_m: 2.0,
        velocity_std_mps: 0.5,
    };
    let gnss_updates = estimator.update_gnss(
        1_005_000,
        [
            Some(GnssSample {
                measurement: gnss_measurement,
                fix_tier: 3,
                pdop_centi: 120,
            }),
            Some(GnssSample {
                measurement: gnss_measurement,
                fix_tier: 3,
                pdop_centi: 140,
            }),
        ],
    )?;

    // Flight-phase logic decides whether each barometer bias may wander. Zero freezes additional
    // process-noise growth; a nonzero value allows tracking slow atmospheric or sensor drift.
    let allow_barometer_bias_drift = false;
    let bias_walk_std_m_per_sqrt_s = if allow_barometer_bias_drift {
        0.02
    } else {
        0.0
    };
    estimator.set_barometer_bias_walk_std(BARO_BUS_1, bias_walk_std_m_per_sqrt_s)?;
    estimator.set_barometer_bias_walk_std(BARO_BUS_2, bias_walk_std_m_per_sqrt_s)?;

    let imu_0_state = estimator.state(IMU_0);
    let imu_1_state = estimator.state(IMU_1);
    let imu_0_uncertainty = estimator.uncertainty(IMU_0);
    let disagreement = estimator.disagreement();
    let redundancy_ready = estimator.redundancy_ready();
    let imu_0_attitude = estimator.imu_status(IMU_0);
    let imu_0_orientation = estimator.orientation_body_to_ned_wxyz(IMU_0);

    println!("IMU 0 priming advanced: {imu_0_priming_advanced}");
    println!("IMU 1 priming advanced: {imu_1_priming_advanced}");
    println!("IMU 0 update advanced: {imu_0_advanced}");
    println!("IMU 1 update advanced: {imu_1_advanced}");
    println!("IMU 0 state: {imu_0_state:?}");
    println!("IMU 1 state: {imu_1_state:?}");
    println!("IMU 0 uncertainty: {imu_0_uncertainty:?}");
    println!("disagreement: {disagreement:?}");
    println!("redundancy ready: {redundancy_ready}");
    println!("IMU 0 body-to-NED quaternion: {imu_0_orientation:?}");
    println!("IMU 0 attitude status: {imu_0_attitude:?}");
    println!("barometer 0 updates: {barometer_0_updates:?}");
    println!("barometer 1 updates: {barometer_1_updates:?}");
    println!("GNSS updates: {gnss_updates:?}");

    Ok(())
}
