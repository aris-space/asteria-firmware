//! Stationary replay with a slowly wandering indoor GNSS altitude.

use asteria_sef_light::{BARO_BUS_1, BARO_BUS_2, IMU_0, IMU_1, PressureMeasurement};
use fw_sensor_carrier_v3::sef::{
    GnssVerticalInput, barometric_pressure_altitude_m, gnss_measurement, imu_measurement,
    new_estimator, select_orientation_imu,
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
        if step.is_multiple_of(833) {
            let time_s = time_us as f32 / 1_000_000.0;
            let height_msl_m = 423.5 + 3.5 * (time_s * core::f32::consts::TAU / 240.0).cos();
            let gnss = gnss_measurement(GnssVerticalInput {
                height_msl_m,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm: 600,
                speed_accuracy_mps: 0.1,
                fix_tier: 3,
                pdop_centi: 150,
            });
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

        if time_us >= 120_000_000 && step.is_multiple_of(833) {
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
fn vertical_quality_handover_keeps_live_orientation_source() {
    assert_eq!(select_orientation_imu(None, IMU_1, false), IMU_1);
    assert_eq!(select_orientation_imu(Some(IMU_1), IMU_0, true), IMU_1);
    assert_eq!(select_orientation_imu(Some(IMU_1), IMU_0, false), IMU_0);
}

#[test]
fn one_metre_lift_changes_msl_height_with_barometer_bias() {
    let mut estimator = new_estimator(2_000.0).unwrap();

    for step in 0..15_000_u64 {
        let time_us = step * 1_200;
        let time_s = time_us as f32 / 1_000_000.0;
        let (height_m, acceleration_up_mps2) = if time_s < 10.0 {
            (0.0, 0.0)
        } else if time_s < 11.0 {
            let phase = (time_s - 10.0) * core::f32::consts::PI;
            (
                0.5 * (1.0 - phase.cos()),
                0.5 * core::f32::consts::PI.powi(2) * phase.cos(),
            )
        } else {
            (1.0, 0.0)
        };
        let imu = imu_measurement(
            [0.0, 0.0, -(9.80665 + acceleration_up_mps2) / 9.80665],
            [0.0; 3],
        );
        estimator.update_imu(IMU_0, time_us, imu).unwrap();
        estimator.update_imu(IMU_1, time_us, imu).unwrap();

        if step.is_multiple_of(833) {
            let height_msl_m = 420.0 + height_m;
            let gnss = gnss_measurement(GnssVerticalInput {
                height_msl_m,
                velocity_down_mps: 0.0,
                vertical_accuracy_mm: 600,
                speed_accuracy_mps: 0.1,
                fix_tier: 3,
                pdop_centi: 150,
            });
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
