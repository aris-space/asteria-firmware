//! A stationary bench replay with one barometer and a plausible accelerometer offset.

use asteria_sef_light::{
    BARO_BUS_1, IMU_0, IMU_1, ImuAttitudeConfig, ImuMeasurement, ImuVerticalizer,
    PressureMeasurement, STANDARD_GRAVITY_MPS2,
};
use fw_sensor_carrier_v3::sef::new_estimator;

#[test]
fn stationary_bias_remains_within_reported_velocity_uncertainty() {
    let mut estimator = new_estimator(2_000.0).unwrap();
    let stationary_imu = ImuMeasurement {
        acceleration_body_mps2: [0.0, 0.0, -STANDARD_GRAVITY_MPS2 - 0.06],
        angular_rate_body_rad_s: [0.0; 3],
    };

    for step in 0..25_000_u64 {
        let time_us = step * 1_200;
        estimator
            .update_imu(IMU_0, time_us, stationary_imu)
            .unwrap();
        estimator
            .update_imu(IMU_1, time_us, stationary_imu)
            .unwrap();
        if step.is_multiple_of(21) {
            estimator
                .update_pressure(
                    time_us,
                    BARO_BUS_1,
                    PressureMeasurement {
                        height_m: 0.0,
                        height_std_m: 3.0,
                    },
                )
                .unwrap();
        }
    }

    let state = estimator.selected_state();
    let velocity_std_mps = estimator
        .selected_uncertainty()
        .velocity_variance_m2_per_s2
        .sqrt();
    assert!(
        state.height_m.abs() < 0.3,
        "unexpected stationary height: {state:?}"
    );
    assert!(
        state.velocity_mps.abs() < 0.2,
        "unexpected stationary velocity: {state:?}"
    );
    assert!(state.velocity_mps.abs() < velocity_std_mps);
    assert!(estimator.redundancy_ready());
}

#[test]
fn measured_stationary_imu_vector_has_a_small_upward_residual() {
    // Representative calibrated-axis reading from the connected stationary board.
    let measurement = ImuMeasurement {
        acceleration_body_mps2: [0.20, -1.14, -9.81],
        angular_rate_body_rad_s: [0.0; 3],
    };
    let mut verticalizer =
        ImuVerticalizer::new(ImuAttitudeConfig::new(2.0, 2_000.0, 10.0, 300).unwrap());
    let mut acceleration_up_mps2 = 0.0;
    for _ in 0..10_000 {
        acceleration_up_mps2 = verticalizer.update(measurement, 0.0012).unwrap();
    }
    assert!((0.04..0.12).contains(&acceleration_up_mps2));
    assert!(!verticalizer.status().flags.initialising());
}
