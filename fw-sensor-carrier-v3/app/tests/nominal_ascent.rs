use asteria_sef_light::{
    BARO_BUS_1, BARO_BUS_2, IMU_0, IMU_1, PressureMeasurement, STANDARD_GRAVITY_MPS2,
};
use fw_sensor_carrier_v3::sef::{
    GnssVerticalInput, barometric_pressure_altitude_m, gnss_measurement, imu_measurement,
    new_estimator,
};

const SAMPLE_PERIOD_US: u64 = 1_200;
const LAUNCH_TIME_S: f32 = 3.0;
const ACCELERATION_UP_MPS2: f32 = 20.0;
const PRESSURE_REFERENCE_MBAR: f32 = 900.0;
const TEMPERATURE_C: f32 = 15.0;
const SCALE_HEIGHT_M: f32 = 29.271 * (TEMPERATURE_C + 273.15);

fn truth(time_us: u64) -> (f32, f32, f32) {
    let flight_time_s = (time_us as f32 / 1_000_000.0 - LAUNCH_TIME_S).max(0.0);
    if flight_time_s == 0.0 {
        (0.0, 0.0, 0.0)
    } else {
        (
            0.5 * ACCELERATION_UP_MPS2 * flight_time_s * flight_time_s,
            ACCELERATION_UP_MPS2 * flight_time_s,
            ACCELERATION_UP_MPS2,
        )
    }
}

#[test]
fn firmware_units_track_a_delayed_aided_ascent() {
    let mut estimator = new_estimator(2_000.0).unwrap();
    for step in 0..=3_500_u64 {
        let time_us = step * SAMPLE_PERIOD_US;
        let (height_m, _, acceleration_up_mps2) = truth(time_us);
        let specific_force_z_g =
            -(STANDARD_GRAVITY_MPS2 + acceleration_up_mps2) / STANDARD_GRAVITY_MPS2;
        let imu = imu_measurement([0.0, 0.0, specific_force_z_g], [0.0; 3]);
        estimator.update_imu(IMU_0, time_us, imu).unwrap();
        estimator.update_imu(IMU_1, time_us, imu).unwrap();

        if step > 83 && step.is_multiple_of(21) {
            let pressure_mbar = PRESSURE_REFERENCE_MBAR * libm::expf(-height_m / SCALE_HEIGHT_M);
            let barometric_height_m = barometric_pressure_altitude_m(pressure_mbar).unwrap();
            let pressure = PressureMeasurement {
                height_m: barometric_height_m,
                height_std_m: 3.0,
            };
            estimator
                .update_pressure(time_us, BARO_BUS_1, pressure)
                .unwrap();
            estimator
                .update_pressure(time_us, BARO_BUS_2, pressure)
                .unwrap();
        }

        // The firmware timestamps the physical GNSS epoch about 100 ms before
        // it can submit that solution to SEF-light.
        if step >= 83 && (step - 83).is_multiple_of(166) {
            let epoch_time_us = (step - 83) * SAMPLE_PERIOD_US;
            let (epoch_height_m, epoch_velocity_mps, _) = truth(epoch_time_us);
            let gnss = gnss_measurement(GnssVerticalInput {
                height_msl_m: 1_600.0 + epoch_height_m,
                velocity_down_mps: -epoch_velocity_mps,
                vertical_accuracy_mm: 1_000,
                speed_accuracy_mps: 0.3,
                fix_tier: 3,
                pdop_centi: 120,
            });
            estimator
                .update_gnss(epoch_time_us, [Some(gnss), Some(gnss)])
                .unwrap();
        }
    }

    let (expected_height_m, expected_velocity_mps, _) = truth(3_500 * SAMPLE_PERIOD_US);
    let state = estimator.selected_state();
    assert!(estimator.redundancy_ready());
    assert!(
        (state.height_m - (1_600.0 + expected_height_m)).abs() < 2.0,
        "height estimate: {} m",
        state.height_m
    );
    assert!(
        (state.velocity_mps - expected_velocity_mps).abs() < 2.0,
        "velocity estimate: {} m/s",
        state.velocity_mps
    );
}
