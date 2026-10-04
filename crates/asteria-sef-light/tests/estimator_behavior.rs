use asteria_sef_light::{
    BARO_BUS_1, BarometerBiasMeasurement, DualVerticalEstimator, EstimatorError, GnssSample,
    GnssSelectorConfig, IMU_0, IMU_1, ImuAttitudeConfig, ImuMeasurement, ImuVerticalizer,
    PressureMeasurement, SelectorConfig, VerticalEstimatorSelectorConfig, VerticalFilter,
    VerticalFilterConfig, VerticalGnssMeasurement, VerticalGnssUpdate, VerticalState,
    VerticalUncertainty,
};

const STATIONARY_IMU: ImuMeasurement = ImuMeasurement {
    acceleration_body_mps2: [0.0, 0.0, -asteria_sef_light::STANDARD_GRAVITY_MPS2],
    angular_rate_body_rad_s: [0.0, 0.0, 0.0],
};

fn filter_config() -> VerticalFilterConfig {
    VerticalFilterConfig::new(0.2, 2.0, [0.01, 0.01], 10.0, 3.0, [10.0, 10.0], 6.0)
        .expect("static vertical filter configuration must be valid")
}

fn estimator<const HISTORY: usize>(
    maximum_aiding_delay_us: u64,
) -> Result<DualVerticalEstimator<HISTORY>, EstimatorError> {
    let attitude = ImuAttitudeConfig::new(0.5, 2_000.0, 20.0, 100)?;
    let selection = SelectorConfig::new(1.0, 0).ok_or(EstimatorError::OutOfRangeInput)?;
    let selector = VerticalEstimatorSelectorConfig::new(0.9, 25.0, 10.0, 20_000, selection)?;
    let gnss = GnssSelectorConfig::new(3, 4.0, 100_000).ok_or(EstimatorError::OutOfRangeInput)?;
    DualVerticalEstimator::new(
        filter_config(),
        [attitude, attitude],
        selector,
        gnss,
        maximum_aiding_delay_us,
    )
}

fn pressure_at_height(height_m: f32) -> PressureMeasurement {
    PressureMeasurement {
        height_m,
        height_std_m: 0.2,
    }
}

fn gnss(height_m: f32, velocity_mps: f32) -> VerticalGnssMeasurement {
    VerticalGnssMeasurement {
        height_m,
        velocity_mps,
        height_std_m: 0.5,
        velocity_std_mps: 0.2,
    }
}

fn update_both<const HISTORY: usize>(estimator: &mut DualVerticalEstimator<HISTORY>, time_us: u64) {
    estimator
        .update_imu(IMU_0, time_us, STATIONARY_IMU)
        .unwrap();
    estimator
        .update_imu(IMU_1, time_us, STATIONARY_IMU)
        .unwrap();
}

#[test]
fn calibrated_magnetic_aiding_reaches_only_its_imu_and_expires() {
    let mut estimator = estimator::<768>(400_000).unwrap();
    update_both(&mut estimator, 10_000);
    estimator
        .update_magnetometer(IMU_0, 15_000, [25_000.0, 0.0, 15_000.0])
        .unwrap();
    update_both(&mut estimator, 20_000);
    assert!(!estimator.imu_status(IMU_0).magnetometer_ignored);
    assert!(estimator.imu_status(IMU_1).magnetometer_ignored);

    update_both(&mut estimator, 300_001);
    assert!(estimator.imu_status(IMU_0).magnetometer_ignored);
}

#[test]
fn magnetic_aiding_limits_stationary_yaw_drift() {
    let config = ImuAttitudeConfig::new(0.5, 2_000.0, 20.0, 100)
        .unwrap()
        .with_magnetic_rejection(20.0)
        .unwrap();
    let mut aided = ImuVerticalizer::new(config);
    let mut unaided = ImuVerticalizer::new(config);
    let biased_gyro = ImuMeasurement {
        angular_rate_body_rad_s: [0.0, 0.0, 0.05],
        ..STATIONARY_IMU
    };
    for _ in 0..1_000 {
        aided
            .update_with_magnetometer(biased_gyro, Some([25_000.0, 0.0, 15_000.0]), 0.01)
            .unwrap();
        unaided.update(biased_gyro, 0.01).unwrap();
    }
    let aided_yaw_component = aided.orientation_body_to_ned_wxyz()[3].abs();
    let unaided_yaw_component = unaided.orientation_body_to_ned_wxyz()[3].abs();
    assert!(aided_yaw_component < unaided_yaw_component * 0.5);
}

fn update_gnss_pair<const HISTORY: usize>(
    estimator: &mut DualVerticalEstimator<HISTORY>,
    fusion_time_us: u64,
    first: VerticalGnssMeasurement,
    second: VerticalGnssMeasurement,
) -> Result<Option<[VerticalGnssUpdate; 2]>, EstimatorError> {
    estimator.update_gnss(
        fusion_time_us,
        [
            Some(GnssSample {
                measurement: first,
                fix_tier: 3,
                pdop_centi: 100,
            }),
            Some(GnssSample {
                measurement: second,
                fix_tier: 3,
                pdop_centi: 120,
            }),
        ],
    )
}

fn assert_state_close(left: VerticalState, right: VerticalState) {
    assert!((left.height_m - right.height_m).abs() < 1.0e-5);
    assert!((left.velocity_mps - right.velocity_mps).abs() < 1.0e-5);
    for index in 0..2 {
        assert!((left.barometer_bias_m[index] - right.barometer_bias_m[index]).abs() < 1.0e-5);
    }
}

fn assert_uncertainty_close(left: VerticalUncertainty, right: VerticalUncertainty) {
    assert!((left.height_variance_m2 - right.height_variance_m2).abs() < 1.0e-5);
    assert!((left.velocity_variance_m2_per_s2 - right.velocity_variance_m2_per_s2).abs() < 1.0e-5);
    assert!(
        (left.height_velocity_covariance_m2_per_s - right.height_velocity_covariance_m2_per_s)
            .abs()
            < 1.0e-5
    );
    for index in 0..2 {
        assert!(
            (left.barometer_bias_variance_m2[index] - right.barometer_bias_variance_m2[index])
                .abs()
                < 1.0e-5
        );
    }
}

fn assert_scores_close(left: [f32; 2], right: [f32; 2]) {
    for (left, right) in left.into_iter().zip(right) {
        assert!((left - right).abs() < 1.0e-6);
    }
}

#[test]
fn prediction_matches_constant_acceleration_kinematics() {
    let mut filter = VerticalFilter::new(filter_config());
    filter.predict(2.0, 3.0).unwrap();

    let state = filter.state();
    assert!((state.height_m - 9.0).abs() < 1.0e-6);
    assert!((state.velocity_mps - 6.0).abs() < 1.0e-6);
}

#[test]
fn gross_gnss_outlier_is_rejected_without_changing_state() {
    let mut filter = VerticalFilter::new(filter_config());
    let before = filter.state();
    let update = filter
        .update_gnss(VerticalGnssMeasurement {
            height_m: 10_000.0,
            velocity_mps: 1_000.0,
            height_std_m: 0.1,
            velocity_std_mps: 0.1,
        })
        .unwrap();

    assert!(!update.height.accepted);
    assert!(!update.velocity.accepted);
    assert_eq!(filter.state(), before);
}

#[test]
fn pressure_updates_only_the_selected_barometer_bias() {
    let mut filter = VerticalFilter::new(filter_config());
    filter.update_gnss(gnss(0.0, 0.0)).unwrap();
    let update = filter
        .update_pressure(BARO_BUS_1, pressure_at_height(8.0))
        .unwrap();

    assert!(update.accepted);
    let state = filter.state();
    assert!(state.barometer_bias_m[0] > 5.0);
    assert!(state.barometer_bias_m[1].abs() < f32::EPSILON);
}

#[test]
fn gnss_bias_reference_changes_height_only_after_pressure_update() {
    let config =
        VerticalFilterConfig::new(0.2, 2.0, [0.01, 0.01], 1_000.0, 3.0, [200.0, 200.0], 6.0)
            .unwrap();
    let mut filter = VerticalFilter::new(config);
    let before = filter.state();
    let update = filter
        .update_barometer_bias(
            BARO_BUS_1,
            BarometerBiasMeasurement {
                bias_m: -20.0,
                std_m: 1.0,
            },
        )
        .unwrap();
    assert!(update.accepted);
    assert_eq!(filter.state().height_m, before.height_m);
    assert_eq!(filter.state().velocity_mps, before.velocity_mps);
    assert!(filter.state().barometer_bias_m[0] < -15.0);
    assert_eq!(filter.state().barometer_bias_m[1], 0.0);

    filter
        .update_pressure(BARO_BUS_1, pressure_at_height(100.0))
        .unwrap();
    assert!(filter.state().height_m > 100.0);
}

#[test]
fn delayed_gnss_bias_reference_matches_chronological_fusion() {
    let mut chronological = estimator::<64>(200_000).unwrap();
    let mut delayed = estimator::<64>(200_000).unwrap();
    let reference = [
        Some(BarometerBiasMeasurement {
            bias_m: -20.0,
            std_m: 1.0,
        }),
        None,
    ];
    for step in 0..=10 {
        let time_us = step * 10_000;
        update_both(&mut chronological, time_us);
        update_both(&mut delayed, time_us);
        if time_us == 50_000 {
            chronological
                .update_barometer_biases(time_us, reference)
                .unwrap();
        }
        if time_us == 70_000 {
            for estimator in [&mut chronological, &mut delayed] {
                estimator
                    .update_pressure(time_us, BARO_BUS_1, pressure_at_height(100.0))
                    .unwrap();
            }
        }
    }
    delayed.update_barometer_biases(50_000, reference).unwrap();
    for imu in [IMU_0, IMU_1] {
        assert_state_close(chronological.state(imu), delayed.state(imu));
        assert_uncertainty_close(chronological.uncertainty(imu), delayed.uncertainty(imu));
    }
}

#[test]
fn stationary_barometer_limits_slow_gnss_height_wander() {
    let config = VerticalFilterConfig::new(
        10.0,
        20.0,
        [0.005, 0.005],
        1_000.0,
        3.0,
        [200.0, 200.0],
        5.0,
    )
    .unwrap();
    let mut filter = VerticalFilter::new(config);
    filter
        .update_barometer_bias(
            BARO_BUS_1,
            BarometerBiasMeasurement {
                bias_m: 316.0 - 440.0,
                std_m: 3.0,
            },
        )
        .unwrap();
    filter
        .update_pressure(
            BARO_BUS_1,
            PressureMeasurement {
                height_m: 316.0,
                height_std_m: 1.5,
            },
        )
        .unwrap();
    let initial_height = filter.state().height_m;

    for step in 1..=3_600 {
        filter.predict(0.0, 0.05).unwrap();
        filter
            .update_pressure(
                BARO_BUS_1,
                PressureMeasurement {
                    height_m: 316.0,
                    height_std_m: 1.5,
                },
            )
            .unwrap();
        let gnss_height_m = 440.0 + 10.0 * step as f32 / 3_600.0;
        if filter.uncertainty().barometer_bias_variance_m2[0] > 1.5 * 1.5 {
            filter
                .update_barometer_bias(
                    BARO_BUS_1,
                    BarometerBiasMeasurement {
                        bias_m: 316.0 - gnss_height_m,
                        std_m: 13.5,
                    },
                )
                .unwrap();
        }
    }
    assert!(
        (filter.state().height_m - initial_height).abs() < 3.0,
        "stationary height moved from {initial_height} to {}",
        filter.state().height_m
    );
}

#[test]
fn delayed_pressure_matches_chronological_fusion() {
    let mut chronological = estimator::<16>(200_000).unwrap();
    let mut delayed = estimator::<16>(200_000).unwrap();

    for step in 0..=10 {
        let time_us = step * 10_000;
        update_both(&mut chronological, time_us);
        update_both(&mut delayed, time_us);
        if time_us == 50_000 {
            chronological
                .update_pressure(50_000, BARO_BUS_1, pressure_at_height(4.0))
                .unwrap();
        }
        if time_us == 70_000 {
            update_gnss_pair(&mut chronological, 70_000, gnss(3.0, 0.2), gnss(3.0, 0.2)).unwrap();
            update_gnss_pair(&mut delayed, 70_000, gnss(3.0, 0.2), gnss(3.0, 0.2)).unwrap();
        }
    }
    delayed
        .update_pressure(50_000, BARO_BUS_1, pressure_at_height(4.0))
        .unwrap();

    for imu in [IMU_0, IMU_1] {
        assert_state_close(chronological.state(imu), delayed.state(imu));
        assert_uncertainty_close(chronological.uncertainty(imu), delayed.uncertainty(imu));
    }
    assert_scores_close(
        chronological.consistency_scores(),
        delayed.consistency_scores(),
    );
}

#[test]
fn delayed_gnss_replays_later_pressure_updates() {
    let mut chronological = estimator::<16>(200_000).unwrap();
    let mut delayed = estimator::<16>(200_000).unwrap();

    for step in 0..=10 {
        let time_us = step * 10_000;
        update_both(&mut chronological, time_us);
        update_both(&mut delayed, time_us);
        if time_us == 50_000 {
            update_gnss_pair(&mut chronological, 50_000, gnss(5.0, 0.5), gnss(5.0, 0.5)).unwrap();
        }
        if time_us == 70_000 {
            chronological
                .update_pressure(70_000, BARO_BUS_1, pressure_at_height(6.0))
                .unwrap();
            delayed
                .update_pressure(70_000, BARO_BUS_1, pressure_at_height(6.0))
                .unwrap();
        }
    }
    update_gnss_pair(&mut delayed, 50_000, gnss(5.0, 0.5), gnss(5.0, 0.5)).unwrap();

    for imu in [IMU_0, IMU_1] {
        assert_state_close(chronological.state(imu), delayed.state(imu));
        assert_uncertainty_close(chronological.uncertainty(imu), delayed.uncertainty(imu));
    }
    assert_scores_close(
        chronological.consistency_scores(),
        delayed.consistency_scores(),
    );
}

#[test]
fn firmware_rate_history_replays_gnss_after_100_ms() {
    let mut chronological = estimator::<256>(120_000).unwrap();
    let mut delayed = estimator::<256>(120_000).unwrap();
    let gnss_time_us = 50 * 1_200;

    for step in 0..=134 {
        let time_us = step * 1_200;
        update_both(&mut chronological, time_us);
        update_both(&mut delayed, time_us);
        if step % 21 == 0 {
            chronological
                .update_pressure(time_us, BARO_BUS_1, pressure_at_height(0.0))
                .unwrap();
            delayed
                .update_pressure(time_us, BARO_BUS_1, pressure_at_height(0.0))
                .unwrap();
        }
        if time_us == gnss_time_us {
            update_gnss_pair(&mut chronological, time_us, gnss(2.0, 0.2), gnss(2.0, 0.2)).unwrap();
        }
    }
    update_gnss_pair(&mut delayed, gnss_time_us, gnss(2.0, 0.2), gnss(2.0, 0.2)).unwrap();

    for imu in [IMU_0, IMU_1] {
        assert_state_close(chronological.state(imu), delayed.state(imu));
        assert_uncertainty_close(chronological.uncertainty(imu), delayed.uncertainty(imu));
    }
    assert_scores_close(
        chronological.consistency_scores(),
        delayed.consistency_scores(),
    );
}

#[test]
fn measurement_older_than_delay_window_is_rejected() {
    let mut estimator = estimator::<64>(20_000).unwrap();
    for step in 0..=10 {
        update_both(&mut estimator, step * 10_000);
    }
    let before = estimator.states();

    let error = estimator
        .update_pressure(10_000, BARO_BUS_1, pressure_at_height(0.0))
        .unwrap_err();
    assert_eq!(error, EstimatorError::MeasurementTooOld);
    assert_eq!(estimator.states(), before);
}

#[test]
fn measurement_at_delay_window_boundary_is_retained() {
    let mut estimator = estimator::<64>(50_000).unwrap();
    for step in 0..=6 {
        update_both(&mut estimator, step * 10_000);
    }

    let updates = estimator
        .update_pressure(10_000, BARO_BUS_1, pressure_at_height(0.0))
        .unwrap();
    assert!(updates.into_iter().all(|update| update.accepted));
}

#[test]
fn invalid_delayed_measurement_does_not_change_filter_state() {
    let mut estimator = estimator::<64>(100_000).unwrap();
    for step in 0..=6 {
        update_both(&mut estimator, step * 10_000);
    }
    let before = estimator.states();
    let mut invalid = pressure_at_height(2.0);
    invalid.height_std_m = 0.0;

    let error = estimator
        .update_pressure(30_000, BARO_BUS_1, invalid)
        .unwrap_err();
    assert_eq!(error, EstimatorError::NonPositiveInput);
    assert_eq!(estimator.states(), before);
}

#[test]
fn exhausted_history_rejects_an_unrecoverable_measurement() {
    let mut estimator = estimator::<4>(1_000_000).unwrap();
    for step in 0..=5 {
        update_both(&mut estimator, step * 10_000);
    }

    let error =
        update_gnss_pair(&mut estimator, 10_000, gnss(0.0, 0.0), gnss(0.0, 0.0)).unwrap_err();
    assert_eq!(error, EstimatorError::MeasurementTooOld);
}

#[test]
fn changing_bias_walk_rejects_measurements_before_the_new_history_base() {
    let mut estimator = estimator::<32>(100_000).unwrap();
    update_both(&mut estimator, 0);
    update_both(&mut estimator, 10_000);
    estimator
        .update_pressure(20_000, BARO_BUS_1, pressure_at_height(0.0))
        .unwrap();
    estimator
        .set_barometer_bias_walk_std(BARO_BUS_1, 0.0)
        .unwrap();

    let before = estimator.states();
    let error = estimator
        .update_pressure(15_000, BARO_BUS_1, pressure_at_height(2.0))
        .unwrap_err();
    assert_eq!(error, EstimatorError::MeasurementTooOld);
    assert_eq!(estimator.states(), before);
}

#[test]
fn rejected_old_gnss_epoch_does_not_change_receiver_selection() {
    let mut baseline = estimator::<4>(1_000_000).unwrap();
    let mut rejected = estimator::<4>(1_000_000).unwrap();
    for step in 0..=5 {
        update_both(&mut baseline, step * 10_000);
        update_both(&mut rejected, step * 10_000);
    }

    let error = rejected
        .update_gnss(
            10_000,
            [
                None,
                Some(GnssSample {
                    measurement: gnss(0.0, 0.0),
                    fix_tier: 3,
                    pdop_centi: 100,
                }),
            ],
        )
        .unwrap_err();
    assert_eq!(error, EstimatorError::MeasurementTooOld);

    update_gnss_pair(&mut baseline, 60_000, gnss(1.0, 0.0), gnss(10.0, 0.0)).unwrap();
    update_gnss_pair(&mut rejected, 60_000, gnss(1.0, 0.0), gnss(10.0, 0.0)).unwrap();
    assert_eq!(rejected.states(), baseline.states());
}

#[test]
fn non_monotonic_imu_sample_is_rejected_without_changing_filter() {
    let mut estimator = estimator::<32>(100_000).unwrap();
    estimator.update_imu(IMU_0, 10_000, STATIONARY_IMU).unwrap();
    estimator.update_imu(IMU_0, 20_000, STATIONARY_IMU).unwrap();
    let before = estimator.state(IMU_0);

    let error = estimator
        .update_imu(IMU_0, 20_000, STATIONARY_IMU)
        .unwrap_err();
    assert_eq!(error, EstimatorError::NonMonotonicImuTimestamp);
    assert_eq!(estimator.state(IMU_0), before);
}

#[test]
fn stale_imu_chain_triggers_selector_handover() {
    let mut estimator = estimator::<64>(100_000).unwrap();
    for step in 0..=2 {
        update_both(&mut estimator, step * 10_000);
    }
    assert_eq!(estimator.selected_imu(), IMU_0);

    for step in 3..=6 {
        estimator
            .update_imu(IMU_1, step * 10_000, STATIONARY_IMU)
            .unwrap();
    }

    assert_eq!(estimator.selected_imu(), IMU_1);
}

#[test]
fn covariance_remains_finite_and_consistent_during_repeated_updates() {
    let mut filter = VerticalFilter::new(filter_config());
    for step in 1..=500 {
        filter.predict(0.2, 0.01).unwrap();
        if step % 5 == 0 {
            filter
                .update_pressure(BARO_BUS_1, pressure_at_height(0.0))
                .unwrap();
        }
        if step % 20 == 0 {
            filter.update_gnss(gnss(0.0, 0.0)).unwrap();
        }
    }

    let uncertainty = filter.uncertainty();
    assert!(uncertainty.height_variance_m2.is_finite() && uncertainty.height_variance_m2 >= 0.0);
    assert!(
        uncertainty.velocity_variance_m2_per_s2.is_finite()
            && uncertainty.velocity_variance_m2_per_s2 >= 0.0
    );
    assert!(
        uncertainty
            .barometer_bias_variance_m2
            .into_iter()
            .all(|variance| variance.is_finite() && variance >= 0.0)
    );
    assert!(uncertainty.height_velocity_covariance_m2_per_s.is_finite());
    assert!(
        uncertainty.height_velocity_covariance_m2_per_s.powi(2)
            <= uncertainty.height_variance_m2 * uncertainty.velocity_variance_m2_per_s2 + 1.0e-5
    );
}

#[test]
fn dual_gnss_fuses_one_completed_epoch_once() {
    let mut estimator = estimator::<32>(100_000).unwrap();
    update_both(&mut estimator, 0);
    update_both(&mut estimator, 10_000);
    let before = estimator.states();

    let updates = estimator
        .update_gnss(
            10_000,
            [
                Some(GnssSample {
                    measurement: gnss(2.0, 0.2),
                    fix_tier: 3,
                    pdop_centi: 100,
                }),
                Some(GnssSample {
                    measurement: gnss(2.2, 0.3),
                    fix_tier: 3,
                    pdop_centi: 120,
                }),
            ],
        )
        .unwrap()
        .expect("the receiver epoch produces one selected solution");
    assert_ne!(estimator.states(), before);
    assert!(
        updates
            .into_iter()
            .all(|update| update.height.accepted && update.velocity.accepted)
    );
}

#[test]
fn dual_gnss_keeps_the_better_receiver_when_solutions_disagree() {
    let mut estimator = estimator::<32>(100_000).unwrap();
    update_both(&mut estimator, 0);
    update_both(&mut estimator, 10_000);

    let updates = update_gnss_pair(&mut estimator, 10_000, gnss(2.0, 0.2), gnss(100.0, 30.0))
        .unwrap()
        .expect("the receiver pair produces one selected solution");

    assert!(
        updates
            .into_iter()
            .all(|update| update.height.accepted && update.velocity.accepted)
    );
    assert!(estimator.selected_state().height_m < 10.0);
}

#[test]
fn zero_history_or_delay_is_rejected() {
    assert!(matches!(
        estimator::<0>(100_000),
        Err(EstimatorError::OutOfRangeInput)
    ));
    assert!(matches!(
        estimator::<8>(0),
        Err(EstimatorError::OutOfRangeInput)
    ));
}

#[test]
fn invalid_imu_attitude_config_is_rejected_at_construction() {
    assert!(matches!(
        ImuAttitudeConfig::new(-0.1, 1.0, 10.0, 1),
        Err(EstimatorError::OutOfRangeInput)
    ));
    assert!(ImuAttitudeConfig::new(0.5, 2_000.0, 90.0, 1).is_ok());
    assert!(matches!(
        ImuAttitudeConfig::new(0.5, 1.0, 90.1, 1),
        Err(EstimatorError::OutOfRangeInput)
    ));
    assert!(matches!(
        ImuAttitudeConfig::new(f32::NAN, 1.0, 10.0, 1),
        Err(EstimatorError::NonFiniteInput)
    ));
}

#[test]
fn invalid_filter_and_selector_configs_are_rejected_at_construction() {
    assert!(matches!(
        VerticalFilterConfig::new(1.0, 0.5, [0.0, 0.0], 1.0, 1.0, [1.0, 1.0], 3.0),
        Err(EstimatorError::OutOfRangeInput)
    ));
    assert!(SelectorConfig::new(-1.0, 0).is_none());
    assert!(GnssSelectorConfig::new(3, 0.0, 0).is_none());

    let selection = SelectorConfig::new(1.0, 0).unwrap();
    assert!(matches!(
        VerticalEstimatorSelectorConfig::new(1.0, 25.0, 10.0, 20_000, selection),
        Err(EstimatorError::OutOfRangeInput)
    ));
}
