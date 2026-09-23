//! Stationary bench replays with barometers and a plausible accelerometer offset.

use asteria_sef_light::{
    BARO_BUS_1, BARO_BUS_2, IMU_0, IMU_1, ImuAttitudeConfig, ImuMeasurement, ImuVerticalizer,
    PressureMeasurement, STANDARD_GRAVITY_MPS2,
};
use fw_sensor_carrier_v3::sef::{
    GnssVerticalInput, barometric_pressure_altitude_m, gnss_measurement, new_estimator,
};

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
        if step == 0 {
            let gnss = gnss_measurement(GnssVerticalInput {
                height_msl_m: 420.0,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm: 600,
                speed_accuracy_mps: 0.1,
                fix_tier: 3,
                pdop_centi: 150,
            });
            estimator.update_gnss(time_us, [Some(gnss), None]).unwrap();
        }
        if step.is_multiple_of(21) {
            estimator
                .update_pressure(
                    time_us,
                    BARO_BUS_1,
                    PressureMeasurement {
                        height_m: barometric_pressure_altitude_m(978.0).unwrap(),
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
        (state.height_m - 420.0).abs() < 0.3,
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
fn raw_pressure_altitudes_before_gnss_converge_to_msl_with_sef_biases() {
    let mut estimator = new_estimator(2_000.0).unwrap();
    let stationary_imu = ImuMeasurement {
        acceleration_body_mps2: [0.0, 0.0, -STANDARD_GRAVITY_MPS2],
        angular_rate_body_rad_s: [0.0; 3],
    };
    let pressure_altitudes = [
        barometric_pressure_altitude_m(977.7).unwrap(),
        barometric_pressure_altitude_m(979.0).unwrap(),
    ];

    for step in 0..30_000_u64 {
        let time_us = step * 1_200;
        estimator
            .update_imu(IMU_0, time_us, stationary_imu)
            .unwrap();
        estimator
            .update_imu(IMU_1, time_us, stationary_imu)
            .unwrap();
        if step.is_multiple_of(21) {
            for (barometer, height_m) in
                [BARO_BUS_1, BARO_BUS_2].into_iter().zip(pressure_altitudes)
            {
                estimator
                    .update_pressure(
                        time_us,
                        barometer,
                        PressureMeasurement {
                            height_m,
                            height_std_m: 1.5,
                        },
                    )
                    .unwrap();
            }
        }
        if step == 8_333 {
            let gnss = gnss_measurement(GnssVerticalInput {
                height_msl_m: 420.0,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm: 600,
                speed_accuracy_mps: 0.1,
                fix_tier: 3,
                pdop_centi: 150,
            });
            let updates = estimator
                .update_gnss(time_us, [Some(gnss), None])
                .unwrap()
                .unwrap();
            assert!(updates[0].height.accepted);
        }
    }

    let state = estimator.selected_state();
    assert!(
        (state.height_m - 420.0).abs() < 0.5,
        "unexpected MSL height: {state:?}"
    );
    for (bias, pressure_height) in state.barometer_bias_m.into_iter().zip(pressure_altitudes) {
        assert!(
            (bias - (pressure_height - 420.0)).abs() < 0.5,
            "unexpected barometer bias: {state:?}"
        );
    }
}

#[test]
fn delayed_first_fix_can_initialize_a_fresh_estimator() {
    let mut estimator = new_estimator(2_000.0).unwrap();
    let gnss = gnss_measurement(GnssVerticalInput {
        height_msl_m: 420.0,
        velocity_down_mps: 0.0,
        vertical_accuracy_mm: 1_000,
        speed_accuracy_mps: 0.15,
        fix_tier: 3,
        pdop_centi: 180,
    });
    let updates = estimator
        .update_gnss(10_000_000, [Some(gnss), None])
        .unwrap()
        .unwrap();
    assert!(updates.iter().all(|update| update.height.accepted));
    assert!((estimator.selected_state().height_m - 420.0).abs() < 0.01);
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
