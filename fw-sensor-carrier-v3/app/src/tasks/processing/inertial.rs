use defmt::trace;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant};
use hermes_can::messages::sensor_data::ImuData as CanImuData;
use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::measurements::ImuSample;
use crate::sensors::{IMU_0, IMU_1, ImuId};
use crate::signals::{self, MagFieldNt};
use crate::tasks::readout::imu::{IMU_ODR_HZ, IMU_TARGET_DT};

// Mean magnetic declination at Gadmen Range, Switzerland (WMMHR-2025, 2026-05-18).
// Used to rotate magnetometer-anchored orientation to true north.
const DECLINATION_DEG: f32 = 3.0 + 28.0 / 60.0 + 56.0 / 3600.0;
const DECLINATION_RAD: f32 = DECLINATION_DEG * (core::f32::consts::PI / 180.0);

const STANDARD_G: f32 = 9.80665;
const GRAVITY: Vector3<f32> = Vector3::new(0.0, 0.0, STANDARD_G);

const TIMEOUT: Duration = Duration::from_millis(100);
/// Treat the magnetometer watch as "fresh" only for this long.
const MAG_FRESH: Duration = Duration::from_millis(100);

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut sub0 = signals::IMU_CHANNELS[IMU_0.index()]
        .subscriber()
        .expect("inertial: subscribe IMU 0");
    let mut sub1 = signals::IMU_CHANNELS[IMU_1.index()]
        .subscriber()
        .expect("inertial: subscribe IMU 1");
    let mut mag_recv = signals::MAG_FIELD_WATCH.anon_receiver();

    let mut fusion = fusion_instance();
    let mut selector = TimeoutSelector::new(TIMEOUT);
    let mut prev_ts: Option<Instant> = None;
    let mut last_mag_ts: Option<Instant> = None;

    let orientation_sender = signals::ORIENTATION_WATCH.sender();
    let inertial_sender = signals::INERTIAL_WATCH.sender();

    loop {
        let sample: ImuSample =
            match select(sub0.next_message_pure(), sub1.next_message_pure()).await {
                Either::First(s) | Either::Second(s) => s,
            };

        if !selector.accept(sample.sensor_id, sample.data.ts) {
            continue;
        }

        let dt = prev_ts
            .map(|p| sample.data.ts.saturating_duration_since(p).as_micros() as f32 / 1e6)
            .unwrap_or(IMU_TARGET_DT);
        prev_ts = Some(sample.data.ts);

        let accel = sample.data.value.accel;
        let gyro = sample.data.value.gyro;

        let gyr_vec = FusionVector::new(gyro.x, gyro.y, gyro.z);
        let acc_vec = FusionVector::new(accel.x, accel.y, accel.z);

        let mag = mag_recv.try_get().and_then(|m| {
            // If we've already seen this exact value within MAG_FRESH, still use it.
            // Otherwise prefer accel-only on first sight.
            let now = Instant::now();
            let fresh = match last_mag_ts {
                Some(t) => now.saturating_duration_since(t) <= MAG_FRESH,
                None => true,
            };
            last_mag_ts = Some(now);
            if fresh { Some(m) } else { None }
        });

        match mag {
            Some(MagFieldNt { x, y, z }) => {
                fusion.update_by_duration_seconds(gyr_vec, acc_vec, FusionVector::new(x, y, z), dt);
            }
            None => {
                fusion.update_no_mag_by_duration_seconds(gyr_vec, acc_vec, dt);
            }
        }

        let q = fusion.quaternion();
        let orientation_mag = UnitQuaternion::new_normalize(Quaternion::new(q.w, q.x, q.y, q.z));
        // Fusion aligns its +x with the measured horizontal field (magnetic north).
        // Mag-NED is the true-NED frame rotated by +declination about z-down, so
        // body -> true-NED is R_z(+declination) * body -> mag-NED.
        let orientation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), DECLINATION_RAD) * orientation_mag;

        // Derived inertial CAN frame: body-frame accel/gyro (units: m/s^2, deg/s)
        // plus NED-rotated accel/gyro with gravity-compensated accel.
        let body_accel_g = Vector3::new(accel.x, accel.y, accel.z);
        let body_angular_vel = Vector3::new(gyro.x, gyro.y, gyro.z);
        let body_accel = body_accel_g * STANDARD_G;
        let inertial_accel = orientation.transform_vector(&body_accel);
        let inertial_gyro = orientation.transform_vector(&body_angular_vel);
        let inertial_accel_comp = inertial_accel + GRAVITY;

        let imu_data = CanImuData {
            acceleration_x: body_accel.x,
            acceleration_y: body_accel.y,
            acceleration_z: body_accel.z,
            angular_velocity_x: body_angular_vel.x,
            angular_velocity_y: body_angular_vel.y,
            angular_velocity_z: body_angular_vel.z,
            acceleration_north: inertial_accel_comp.x,
            acceleration_east: inertial_accel_comp.y,
            acceleration_down: inertial_accel_comp.z,
            angular_velocity_north: inertial_gyro.x,
            angular_velocity_east: inertial_gyro.y,
            angular_velocity_down: inertial_gyro.z,
        };

        orientation_sender.send(orientation);
        inertial_sender.send(imu_data);
        trace!("inertial: dt={} s", dt);
    }
}

fn fusion_instance() -> Fusion {
    let mut s = FusionAhrsSettings::new();
    s.gyr_range = 2_000.0;
    s.gain = 2.0;
    s.acc_rejection = 10.0;
    s.recovery_trigger_period = 300; // ~8 s
    s.mag_rejection = 10.0;
    s.convention = imu_fusion::FusionConvention::NED;
    Fusion::new(IMU_ODR_HZ, s)
}

struct TimeoutSelector {
    primary: ImuId,
    last_update: Instant,
    timeout: Duration,
}

impl TimeoutSelector {
    fn new(timeout: Duration) -> Self {
        Self {
            primary: IMU_0,
            last_update: Instant::now(),
            timeout,
        }
    }

    fn accept(&mut self, id: ImuId, ts: Instant) -> bool {
        if self.primary == id || self.last_update.elapsed() > self.timeout {
            self.primary = id;
            self.last_update = ts;
            true
        } else {
            false
        }
    }
}
