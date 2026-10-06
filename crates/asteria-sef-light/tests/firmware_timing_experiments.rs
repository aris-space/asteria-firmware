//! Host-side timing and backlog experiments using the sensor carrier's configured rates.

use std::{collections::VecDeque, mem::size_of, time::Instant};

use asteria_sef_light::{
    BARO_BUS_1, BARO_BUS_2, DualVerticalEstimator, EstimatorError, IMU_0, IMU_1, ImuAttitudeConfig,
    ImuMeasurement, PressureMeasurement, STANDARD_GRAVITY_MPS2, SelectorConfig,
    VerticalEstimatorSelectorConfig, VerticalFilterConfig, VerticalGnssMeasurement,
};

const IMU_PERIOD_US: u64 = 1_200; // Approximately 833 Hz.
const BAROMETER_PERIOD_STEPS: u64 = 21; // 25.2 ms, approximately 40 Hz.
const STATIONARY_IMU: ImuMeasurement = ImuMeasurement {
    acceleration_body_mps2: [0.0, 0.0, -STANDARD_GRAVITY_MPS2],
    angular_rate_body_rad_s: [0.0; 3],
};

fn estimator<const HISTORY: usize>() -> DualVerticalEstimator<HISTORY> {
    let filter =
        VerticalFilterConfig::new(0.5, 5.0, [0.01, 0.01], 20.0, 5.0, [15.0, 15.0], 6.0).unwrap();
    let attitude = ImuAttitudeConfig::new(0.5, 2_000.0, 15.0, 100).unwrap();
    let selection = SelectorConfig::new(2.0, 100_000).unwrap();
    let selector =
        VerticalEstimatorSelectorConfig::new(0.95, 25.0, 10.0, 20_000, selection).unwrap();
    DualVerticalEstimator::new(filter, [attitude, attitude], selector, 240_000).unwrap()
}

fn gnss_measurement() -> VerticalGnssMeasurement {
    VerticalGnssMeasurement {
        height_m: 2.0,
        velocity_mps: 0.2,
        height_std_m: 1.5,
        velocity_std_mps: 0.3,
    }
}

fn delayed_gnss<const HISTORY: usize>(
    delay_steps: u64,
) -> (Result<(), EstimatorError>, std::time::Duration) {
    let mut estimator = estimator::<HISTORY>();
    let fusion_time_us = 50 * IMU_PERIOD_US;
    for step in 0..=50 + delay_steps {
        let time_us = step * IMU_PERIOD_US;
        estimator
            .update_imu(IMU_0, time_us, STATIONARY_IMU)
            .unwrap();
        estimator
            .update_imu(IMU_1, time_us, STATIONARY_IMU)
            .unwrap();
        if step.is_multiple_of(BAROMETER_PERIOD_STEPS) {
            let measurement = PressureMeasurement {
                height_m: 0.0,
                height_std_m: 0.7,
            };
            estimator
                .update_pressure(time_us, BARO_BUS_1, measurement)
                .unwrap();
            estimator
                .update_pressure(time_us, BARO_BUS_2, measurement)
                .unwrap();
        }
    }
    let started = Instant::now();
    let result = estimator
        .fuse_gnss(fusion_time_us, gnss_measurement())
        .map(|_| ());
    (result, started.elapsed())
}

#[test]
fn history_capacity_matches_aiding_latency() {
    let (at_100_ms, host_100_ms) = delayed_gnss::<256>(84);
    let (at_180_ms_small, _) = delayed_gnss::<256>(150);
    let (at_180_ms_large, host_180_ms) = delayed_gnss::<512>(150);

    println!(
        "host history size: 256 entries={} bytes, 512 entries={} bytes; host delayed GNSS replay: 100 ms={host_100_ms:?}, 180 ms={host_180_ms:?}",
        size_of::<DualVerticalEstimator<256>>(),
        size_of::<DualVerticalEstimator<512>>()
    );
    assert!(at_100_ms.is_ok());
    assert_eq!(at_180_ms_small, Err(EstimatorError::MeasurementTooOld));
    assert!(at_180_ms_large.is_ok());
}

#[test]
fn one_imu_channel_recovers_after_a_150_ms_backlog() {
    const CHANNEL_CAPACITY: usize = 64;
    let mut estimator = estimator::<256>();
    let mut delayed_imu_0 = VecDeque::with_capacity(CHANNEL_CAPACITY);
    let mut discarded = 0;

    for step in 0..=220 {
        let time_us = step * IMU_PERIOD_US;
        estimator
            .update_imu(IMU_1, time_us, STATIONARY_IMU)
            .unwrap();
        if step <= 80 {
            estimator
                .update_imu(IMU_0, time_us, STATIONARY_IMU)
                .unwrap();
        } else if delayed_imu_0.len() == CHANNEL_CAPACITY {
            delayed_imu_0.pop_front();
            discarded += 1;
        }
        if step > 80 {
            delayed_imu_0.push_back(time_us);
        }
    }

    assert_eq!(estimator.selected_imu(), IMU_1);
    let oldest_retained_time_us = *delayed_imu_0.front().unwrap();
    for time_us in delayed_imu_0 {
        estimator
            .update_imu(IMU_0, time_us, STATIONARY_IMU)
            .unwrap();
    }
    println!(
        "backlog: discarded {discarded} IMU 0 samples; oldest retained sample was {oldest_retained_time_us} us"
    );
    assert_eq!(discarded, 76);
    assert_eq!(
        estimator.last_imu_sample_time_us(IMU_0),
        Some(220 * IMU_PERIOD_US)
    );
    assert!(
        estimator
            .states()
            .iter()
            .all(|state| state.height_m.is_finite() && state.velocity_mps.is_finite())
    );
}

#[test]
fn fifo_bursts_with_delayed_gnss_remain_processable() {
    const SAMPLES_PER_IMU_BURST: u64 = 13; // Firmware FIFO watermark is 26 accel/gyro entries.
    let mut estimator = estimator::<256>();
    let mut worst_burst = std::time::Duration::ZERO;
    let mut gnss_fixes = 0;

    for burst in 0..40 {
        let started = Instant::now();
        for offset in 0..SAMPLES_PER_IMU_BURST {
            let step = burst * SAMPLES_PER_IMU_BURST + offset;
            let time_us = step * IMU_PERIOD_US;
            estimator
                .update_imu(IMU_0, time_us, STATIONARY_IMU)
                .unwrap();
            estimator
                .update_imu(IMU_1, time_us, STATIONARY_IMU)
                .unwrap();
            if step.is_multiple_of(BAROMETER_PERIOD_STEPS) {
                let measurement = PressureMeasurement {
                    height_m: 0.0,
                    height_std_m: 0.7,
                };
                estimator
                    .update_pressure(time_us, BARO_BUS_1, measurement)
                    .unwrap();
                estimator
                    .update_pressure(time_us, BARO_BUS_2, measurement)
                    .unwrap();
            }
        }
        if burst >= 8 && burst.is_multiple_of(8) {
            let newest_sample_time_us = ((burst + 1) * SAMPLES_PER_IMU_BURST - 1) * IMU_PERIOD_US;
            estimator
                .fuse_gnss(newest_sample_time_us - 100_800, gnss_measurement())
                .unwrap();
            gnss_fixes += 1;
        }
        worst_burst = worst_burst.max(started.elapsed());
    }

    println!(
        "host worst 26-sample FIFO burst with aiding: {worst_burst:?}; GNSS fixes {gnss_fixes}"
    );
    assert_eq!(gnss_fixes, 4);
    assert!(
        estimator
            .states()
            .iter()
            .all(|state| state.height_m.is_finite() && state.velocity_mps.is_finite())
    );
}
