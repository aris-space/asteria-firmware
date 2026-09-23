//! Stationary replay with a slowly wandering indoor GNSS altitude.

use asteria_sef_light::{BARO_BUS_1, BARO_BUS_2, IMU_0, IMU_1, PressureMeasurement};
use fw_sensor_carrier_v3::sef::{
    BarometerBiasTracker, GnssVerticalInput, barometric_pressure_altitude_m,
    correlated_gnss_height_std_m, gnss_height_std_m, gnss_measurement, imu_measurement,
    new_estimator, weaker_gnss_disagreement_floor_m,
};

#[test]
fn stationary_barometers_reject_correlated_gnss_height_wander() {
    let mut estimator = new_estimator(2_000.0).unwrap();
    let imu = imu_measurement([0.0, 0.0, -1.006], [0.0; 3]);
    let mut lowest = f32::INFINITY;
    let mut highest = f32::NEG_INFINITY;

    for step in 0..200_000_u64 {
        let time_us = step * 1_200;
        estimator.update_imu(IMU_0, time_us, imu).unwrap();
        estimator.update_imu(IMU_1, time_us, imu).unwrap();

        // A 7 m peak-to-peak wander over four minutes is representative of
        // the stationary indoor receiver trace. Its reported vAcc was <1 m.
        if step.is_multiple_of(42) {
            let time_s = time_us as f32 / 1_000_000.0;
            let height_msl_m = 423.5 + 3.5 * (time_s * core::f32::consts::TAU / 240.0).cos();
            let mut gnss = gnss_measurement(GnssVerticalInput {
                height_msl_m,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm: 600,
                speed_accuracy_mps: 0.1,
                fix_tier: 3,
                pdop_centi: 150,
            });
            gnss.measurement.height_std_m *= libm::sqrtf(20.0);
            estimator.update_gnss(time_us, [None, Some(gnss)]).unwrap();
        }
        if step.is_multiple_of(21) {
            for barometer in [BARO_BUS_1, BARO_BUS_2] {
                estimator
                    .update_pressure(
                        time_us,
                        barometer,
                        PressureMeasurement {
                            height_m: barometric_pressure_altitude_m(978.0).unwrap(),
                            height_std_m: 3.0,
                        },
                    )
                    .unwrap();
            }
        }

        if time_us >= 120_000_000 && step.is_multiple_of(42) {
            let height_m = estimator.selected_state().height_m;
            lowest = lowest.min(height_m);
            highest = highest.max(height_m);
        }
    }

    assert!(estimator.redundancy_ready());
    let final_msl_m = estimator.selected_state().height_m;
    assert!(
        (final_msl_m - 423.5).abs() < 1.0,
        "MSL datum did not converge: {final_msl_m}"
    );
    assert!(
        highest - lowest < 1.0,
        "stationary estimate wandered {} m, from {lowest} to {highest}",
        highest - lowest
    );
}

#[test]
fn stationary_height_survives_divergent_barometer_drift() {
    let mut estimator = new_estimator(2_000.0).unwrap();
    let mut bias_tracker = BarometerBiasTracker::default();
    let imu = imu_measurement([0.0, 0.0, -1.02], [0.0; 3]);
    let mut accepted_heights = 0;
    let mut lowest = f32::INFINITY;
    let mut highest = f32::NEG_INFINITY;
    let mut early_peak_height_error_m = 0.0_f32;
    let mut velocity_sum = 0.0;
    let mut velocity_samples = 0;

    for step in 0..200_000_u64 {
        let time_us = step * 1_200;
        let time_s = time_us as f32 / 1_000_000.0;
        estimator.update_imu(IMU_0, time_us, imu).unwrap();
        estimator.update_imu(IMU_1, time_us, imu).unwrap();

        if step.is_multiple_of(21) {
            // A stationary board showed approximately these two fast
            // pressure-altitude rates while its GNSS height stayed near MSL.
            for (index, (barometer, height_m)) in [
                (BARO_BUS_1, 338.0 + 0.55 * time_s),
                (BARO_BUS_2, 311.0 + 0.38 * time_s),
            ]
            .into_iter()
            .enumerate()
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
                if let Some(walk_std) = bias_tracker.observe(index, time_us, height_m, 0.0) {
                    estimator
                        .set_barometer_bias_walk_std(barometer, walk_std)
                        .unwrap();
                }
            }
        }
        if step.is_multiple_of(42) {
            let mut gnss = gnss_measurement(GnssVerticalInput {
                height_msl_m: 429.0,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm: 1_800,
                speed_accuracy_mps: 0.15,
                fix_tier: 3,
                pdop_centi: 390,
            });
            gnss.measurement.height_std_m *= libm::sqrtf(20.0);
            let updates = estimator
                .update_gnss(time_us, [None, Some(gnss)])
                .unwrap()
                .unwrap();
            accepted_heights +=
                u32::from(updates[estimator.selected_imu().index()].height.accepted);
            if (5.0..30.0).contains(&time_s) {
                early_peak_height_error_m = early_peak_height_error_m
                    .max((estimator.selected_state().height_m - 429.0).abs());
            }
            if time_s >= 120.0 {
                let height_m = estimator.selected_state().height_m;
                lowest = lowest.min(height_m);
                highest = highest.max(height_m);
                velocity_sum += estimator.selected_state().velocity_mps;
                velocity_samples += 1;
            }
        }
    }

    let state = estimator.selected_state();
    assert!(
        early_peak_height_error_m < 2.0,
        "fast barometer drift moved the estimate {early_peak_height_error_m} m before 30 s"
    );
    assert!(
        accepted_heights > 1100,
        "GNSS corrections stopped: {accepted_heights}"
    );
    assert!(
        (state.height_m - 429.0).abs() < 2.0,
        "stationary height drifted: {state:?}"
    );
    assert!(
        highest - lowest < 2.0,
        "stationary height ranged from {lowest} to {highest}"
    );
    assert!(
        state.velocity_mps.abs() < 0.1,
        "stationary velocity: {state:?}"
    );
    assert!(
        (velocity_sum / velocity_samples as f32).abs() < 0.025,
        "mean stationary velocity: {}",
        velocity_sum / velocity_samples as f32
    );
}

#[test]
fn weak_receiver_after_good_fix_does_not_drag_stationary_height_far() {
    let mut estimator = new_estimator(2_000.0).unwrap();
    let imu = imu_measurement([0.0, 0.0, -1.006], [0.0; 3]);
    let mut height_before_dropout = 0.0;
    let mut height_after_weak_fix = 0.0;
    let mut weak_updates = 0_u32;
    let mut weak_height_accepted = 0_u32;
    let mut weak_velocity_accepted = 0_u32;
    let disagreement_floor = weaker_gnss_disagreement_floor_m(
        [384.0, 415.0],
        [0.0; 2],
        [0; 2],
        [gnss_height_std_m(3_700, 517), gnss_height_std_m(1_200, 313)],
    );

    for step in 0..400_000_u64 {
        let time_us = step * 1_200;
        estimator.update_imu(IMU_0, time_us, imu).unwrap();
        estimator.update_imu(IMU_1, time_us, imu).unwrap();
        if step.is_multiple_of(21) {
            for (barometer, height_m) in [(BARO_BUS_1, 342.0), (BARO_BUS_2, 330.0)] {
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
        if step.is_multiple_of(42) {
            let good_receiver = time_us < 120_000_000 || time_us >= 420_000_000;
            let (source, height, vacc, pdop) = if good_receiver {
                (1, 415.0, 1_200, 313)
            } else {
                (0, 384.0, 3_700, 517)
            };
            let mut fix = gnss_measurement(GnssVerticalInput {
                height_msl_m: height,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm: vacc,
                speed_accuracy_mps: 0.2,
                fix_tier: 3,
                pdop_centi: pdop,
            });
            fix.measurement.height_std_m = correlated_gnss_height_std_m(
                fix.measurement.height_std_m.max(disagreement_floor[source]),
                50_000,
            );
            let mut fixes = [None, None];
            fixes[source] = Some(fix);
            let updates = estimator.update_gnss(time_us, fixes).unwrap().unwrap();
            if !good_receiver {
                weak_updates += 1;
                let update = &updates[estimator.selected_imu().index()];
                weak_height_accepted += u32::from(update.height.accepted);
                weak_velocity_accepted += u32::from(update.velocity.accepted);
            }
            if time_us < 120_000_000 {
                height_before_dropout = estimator.selected_state().height_m;
            } else if time_us < 420_000_000 {
                height_after_weak_fix = estimator.selected_state().height_m;
            }
        }
    }

    assert!(
        (height_after_weak_fix - height_before_dropout).abs() < 5.0,
        "weak receiver pulled stationary MSL from {height_before_dropout} to {height_after_weak_fix}"
    );
    assert_eq!(weak_height_accepted, weak_updates);
    assert_eq!(weak_velocity_accepted, weak_updates);
    let height_after_good_fix_returns = estimator.selected_state().height_m;
    assert!(
        (height_after_good_fix_returns - 415.0).abs() < 2.0,
        "height did not recover with the better receiver: {height_after_good_fix_returns}"
    );
}

#[test]
fn one_metre_lift_changes_msl_height_with_barometer_bias() {
    let mut estimator = new_estimator(2_000.0).unwrap();

    for step in 0..15_000_u64 {
        let time_us = step * 1_200;
        let time_s = time_us as f32 / 1_000_000.0;
        let (height_m, velocity_up_mps, acceleration_up_mps2) = if time_s < 10.0 {
            (0.0, 0.0, 0.0)
        } else if time_s < 11.0 {
            let phase = (time_s - 10.0) * core::f32::consts::PI;
            (
                0.5 * (1.0 - phase.cos()),
                0.5 * core::f32::consts::PI * phase.sin(),
                0.5 * core::f32::consts::PI.powi(2) * phase.cos(),
            )
        } else {
            (1.0, 0.0, 0.0)
        };
        let imu = imu_measurement(
            [0.0, 0.0, -(9.80665 + acceleration_up_mps2) / 9.80665],
            [0.0; 3],
        );
        estimator.update_imu(IMU_0, time_us, imu).unwrap();
        estimator.update_imu(IMU_1, time_us, imu).unwrap();

        if step.is_multiple_of(42) {
            let height_msl_m = 420.0 + height_m;
            let mut gnss = gnss_measurement(GnssVerticalInput {
                height_msl_m,
                velocity_down_mps: -velocity_up_mps,
                vertical_accuracy_mm: 600,
                speed_accuracy_mps: 0.1,
                fix_tier: 3,
                pdop_centi: 150,
            });
            gnss.measurement.height_std_m *= libm::sqrtf(20.0);
            estimator.update_gnss(time_us, [None, Some(gnss)]).unwrap();
        }
        if step.is_multiple_of(21) {
            let pressure = PressureMeasurement {
                height_m: barometric_pressure_altitude_m(978.0).unwrap() + height_m,
                height_std_m: 3.0,
            };
            estimator
                .update_pressure(time_us, BARO_BUS_1, pressure)
                .unwrap();
            estimator
                .update_pressure(time_us, BARO_BUS_2, pressure)
                .unwrap();
        }
    }

    let altitude_msl_m = estimator.selected_state().height_m;
    assert!(
        (altitude_msl_m - 421.0).abs() < 0.2,
        "lift estimate: {altitude_msl_m}"
    );
}

#[test]
fn stationary_consistency_selects_the_better_imu_and_fails_over_if_it_stales() {
    let mut estimator = new_estimator(2_000.0).unwrap();
    let imu0 = imu_measurement([0.0, 0.0, -1.006], [0.0; 3]);
    let imu1 = imu_measurement([0.0, 0.0, -1.03], [0.0; 3]);
    let mut settled_switches = 0;
    let mut previous = IMU_1;

    for step in 0..25_000_u64 {
        let time_us = step * 1_200;
        // Begin with IMU_1 selected, then let common aiding compare the chains.
        estimator.update_imu(IMU_1, time_us, imu1).unwrap();
        estimator.update_imu(IMU_0, time_us, imu0).unwrap();
        if step == 1 {
            assert_eq!(estimator.selected_imu(), IMU_1);
        }
        if step.is_multiple_of(833) {
            let gnss = gnss_measurement(GnssVerticalInput {
                height_msl_m: 420.0,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm: 700,
                speed_accuracy_mps: 0.15,
                fix_tier: 3,
                pdop_centi: 150,
            });
            estimator.update_gnss(time_us, [None, Some(gnss)]).unwrap();
        }
        if step.is_multiple_of(21) {
            for (barometer, pressure_mbar) in [(BARO_BUS_1, 977.0), (BARO_BUS_2, 978.5)] {
                estimator
                    .update_pressure(
                        time_us,
                        barometer,
                        PressureMeasurement {
                            height_m: barometric_pressure_altitude_m(pressure_mbar).unwrap(),
                            height_std_m: 1.5,
                        },
                    )
                    .unwrap();
            }
        }
        let selected = estimator.selected_imu();
        if selected != previous {
            if time_us >= 1_000_000 {
                settled_switches += 1;
            }
            previous = selected;
        }
    }

    assert_eq!(estimator.selected_imu(), IMU_0);
    assert_eq!(
        settled_switches, 1,
        "selector switched {settled_switches} times after startup"
    );

    // A missing selected IMU must hand over without waiting for the quality dwell.
    for step in 25_000..25_101_u64 {
        estimator.update_imu(IMU_1, step * 1_200, imu1).unwrap();
    }
    assert_eq!(estimator.selected_imu(), IMU_1);
}

#[test]
fn calibrated_magnetic_field_reaches_one_attitude_chain() {
    let mut estimator = new_estimator(2_000.0).unwrap();
    let stationary_imu = imu_measurement([0.0, 0.0, -1.0], [0.0; 3]);

    for step in 0..5_000_u64 {
        let time_us = step * 1_200;
        if step.is_multiple_of(42) {
            estimator
                .update_magnetometer(IMU_0, time_us, [20_000.0, 0.0, 40_000.0])
                .unwrap();
        }
        estimator
            .update_imu(IMU_0, time_us, stationary_imu)
            .unwrap();
        estimator
            .update_imu(IMU_1, time_us, stationary_imu)
            .unwrap();
    }

    let status = estimator.imu_status(IMU_0);
    assert!(
        !status.magnetometer_ignored,
        "magnetic sample was ignored: {status:?}"
    );
    assert!(estimator.imu_status(IMU_1).magnetometer_ignored);
}
