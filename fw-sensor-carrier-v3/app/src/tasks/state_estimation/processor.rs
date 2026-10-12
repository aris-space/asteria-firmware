use asteria_sef_core::{
    BarometerInput, GeodeticPosition, GeodeticReference, GnssInput, ImuInput, MagnetometerInput,
    StateEstimator, UpdateError,
};
use asteria_sef_light::STANDARD_GRAVITY_MPS2;
use embassy_time::Instant;
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use super::{config, position_reference};
use crate::calibration;
use crate::sensors::ImuId;
use crate::signals;
use crate::types::{BaroSample, GnssSample, ImuSample, MagSample, SefLogSample};

pub struct Processor<E> {
    estimator: E,
    reference: GeodeticReference,
    newest_imu: Instant,
    last_publish: Option<Instant>,
}

impl<E: StateEstimator> Processor<E> {
    pub fn new(estimator: E) -> Self {
        Self {
            estimator,
            reference: position_reference(),
            newest_imu: Instant::MIN,
            last_publish: None,
        }
    }

    pub fn update_imu(&mut self, sample: ImuSample) -> Result<(), UpdateError> {
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;

        self.newest_imu = self.newest_imu.max(sample.ts);

        let acceleration_body_mps2 =
            [sample.accel.x, sample.accel.y, sample.accel.z].map(|g| g * STANDARD_GRAVITY_MPS2);
        let angular_rate_body_rad_s =
            [sample.gyro.x, sample.gyro.y, sample.gyro.z].map(|dps| dps * DEG_TO_RAD);

        let input = ImuInput {
            timestamp_us: sample.ts.as_micros(),
            sensor_index: sample.src.index(),
            acceleration_body_mps2,
            angular_rate_body_rad_s,
        };
        self.estimator.update_imu(input)
    }

    pub fn update_mag(&mut self, sample: MagSample) -> Result<(), UpdateError> {
        let field_body = [sample.x, sample.y, sample.z];
        let field_strength_nt = libm::sqrtf(field_body.iter().map(|value| value * value).sum());
        let calibration = calibration::mag::CAL.applied(sample.src);
        if !calibration.correction.accepts_field(field_strength_nt) {
            return Ok(());
        }

        // Magnetometers use the index of their paired IMU.
        let input = MagnetometerInput {
            timestamp_us: sample.ts.as_micros(),
            sensor_index: sample.src.index(),
            field_body,
            reference_field_ned: config::LOCAL_MAGNETIC_FIELD_NED_NT,
            direction_noise_std: config::MAG_DIRECTION_STD,
        };
        self.estimator.update_magnetometer(input)
    }

    pub fn update_baro(&mut self, sample: BaroSample) -> Result<(), UpdateError> {
        let input = BarometerInput {
            timestamp_us: sample.ts.as_micros(),
            sensor_index: sample.src.index(),
            height_m: sample.pressure_altitude_m(),
            height_std_m: config::BARO_HEIGHT_STD_M,
            pressure_hpa: sample.pressure_mbar,
            reference_pressure_hpa: config::BARO_REFERENCE_PRESSURE_HPA,
            pressure_std_hpa: config::BARO_PRESSURE_STD_HPA,
        };
        self.estimator.update_barometer(input)
    }

    /// Receiver selection is handled by the estimator.
    pub fn update_gnss(&mut self, fix: GnssSample) -> Result<(), UpdateError> {
        let pvt = &fix.pvt;
        if !pvt.has_3d_fix() {
            return Ok(());
        }

        let position_ned_m = self.reference.project(GeodeticPosition {
            latitude_deg: pvt.latitude_deg,
            longitude_deg: pvt.longitude_deg,
            height_msl_m: pvt.height_msl_m,
        });
        let velocity_ned_mps = [
            pvt.velocity_north_mps,
            pvt.velocity_east_mps,
            pvt.velocity_down_mps,
        ];
        let height_m = -position_ned_m[2];
        let vertical_velocity_mps = -pvt.velocity_down_mps;

        // hAcc covers the horizontal error as a whole, split across north and east.
        let horizontal_accuracy_m = pvt.horizontal_accuracy_mm as f32 / 1_000.0;
        let vertical_accuracy_m = pvt.vertical_accuracy_mm as f32 / 1_000.0;
        let horizontal_std_m =
            (horizontal_accuracy_m / core::f32::consts::SQRT_2).max(config::GNSS_MIN_STD);
        let height_std_m =
            (vertical_accuracy_m * config::GNSS_HEIGHT_STD_SCALE).max(config::GNSS_MIN_STD);
        let speed_std_mps = pvt.speed_accuracy_mps.max(config::GNSS_MIN_STD);
        let vertical_speed_std_mps =
            (pvt.speed_accuracy_mps * config::GNSS_SPEED_STD_SCALE).max(config::GNSS_MIN_STD);

        let input = GnssInput {
            timestamp_us: fix.ts.as_micros(),
            sensor_index: fix.src.index(),
            height_m,
            vertical_velocity_mps,
            height_std_m,
            vertical_velocity_std_mps: vertical_speed_std_mps,
            position_ned_m,
            velocity_ned_mps,
            position_std_m: [horizontal_std_m, horizontal_std_m, height_std_m],
            velocity_std_mps: [speed_std_mps, speed_std_mps, vertical_speed_std_mps],
        };
        self.estimator.update_gnss(input)
    }
}

impl Processor<config::Estimator> {
    pub fn publish(&mut self) {
        // Spaced in sample time: the IMU samples arrive in FIFO batches.
        let newest_imu = self.newest_imu;
        if self
            .last_publish
            .is_some_and(|last| newest_imu.saturating_duration_since(last) < config::PUBLISH_PERIOD)
        {
            return;
        }
        self.last_publish = Some(newest_imu);

        let now = Instant::now();
        let mut navigation_state = self.estimator.navigation_state();
        let timestamp = Instant::from_micros(navigation_state.time_us);
        if now.saturating_duration_since(timestamp) > config::IMU_FRESH {
            return;
        }

        navigation_state.orientation_body_to_ned_wxyz =
            true_north(navigation_state.orientation_body_to_ned_wxyz);
        signals::STATE_ESTIMATE.sender().send(navigation_state);

        let estimator = &self.estimator;
        let selected_imu = estimator.selected_imu();
        signals::SELECTED_IMU
            .sender()
            .send(ImuId::ALL[selected_imu]);
        let scores = estimator.consistency_scores();
        for imu in ImuId::ALL {
            let index = imu.index();
            let vertical_state = estimator.state(index);
            let uncertainty = estimator.uncertainty(index);

            let height_std_m = libm::sqrtf(uncertainty.height_variance_m2);
            let velocity_std_mps = libm::sqrtf(uncertainty.velocity_variance_m2_per_s2);
            let barometer_bias_std_m = uncertainty.barometer_bias_variance_m2.map(libm::sqrtf);
            let orientation_body_to_ned_wxyz =
                true_north(estimator.orientation_body_to_ned_wxyz(index));

            signals::submit_state(SefLogSample {
                ts: timestamp,
                imu,
                selected: index == selected_imu,
                msl_ready: height_std_m < config::MSL_REFERENCED_HEIGHT_STD_M,
                redundancy_ready: estimator.redundancy_ready(),
                height_msl_m: vertical_state.height_m,
                velocity_mps: vertical_state.velocity_mps,
                barometer_bias_m: vertical_state.barometer_bias_m,
                height_std_m,
                velocity_std_mps,
                barometer_bias_std_m,
                consistency_score: scores[index],
                orientation_body_to_ned_wxyz,
            });
        }
    }
}

/// Rotates about down by the declination, from magnetic to true north.
fn true_north(body_to_ned_wxyz: [f32; 4]) -> [f32; 4] {
    let [w, x, y, z] = body_to_ned_wxyz;
    let declination = UnitQuaternion::from_axis_angle(
        &Vector3::z_axis(),
        config::MAGNETIC_DECLINATION_DEG.to_radians(),
    );
    let orientation = declination * UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z));
    [orientation.w, orientation.i, orientation.j, orientation.k]
}
