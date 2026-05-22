use defmt::trace;
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant};
use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::measurements::{ImuSample, Inertial, MagSample, Orientation};
use crate::sensors::{IMU_0, IMU_1, ImuId};
use crate::signals;
use crate::tasks::readout::imu::{IMU_ODR_HZ, IMU_TARGET_DT};

// Taken from https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm
// Calculated for Gadmen Range, Switzerland
// Model Used:  WMMHR-2025
// Latitude:    46° 44' 59" N
// Longitude:   8° 23' 11" E
// Elevation:   1606.0 m Mean Sea Level
//
// 2026-05-18   Declination: 3° 28' 56" E
//              changing by +0° 7' 24"/yr
//              uncertainty ±0° 20' (1σ)
const DECLINATION_DEG: f32 = 3.0 + 28.0 / 60.0 + 56.0 / 3600.0;
const DECLINATION_RAD: f32 = DECLINATION_DEG.to_radians();

const STANDARD_G: f32 = 9.80665;
const GRAVITY: Vector3<f32> = Vector3::new(0.0, 0.0, STANDARD_G);

const TIMEOUT: Duration = Duration::from_millis(100);
/// Treat the magnetometer watch as "fresh" only for this long.
const MAG_FRESH: Duration = Duration::from_millis(100);

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut sub0 = signals::IMU_CHANNELS[IMU_0.index()]
        .subscriber()
        .expect("too many subs on IMU_CHANNELS; increase SUBS");
    let mut sub1 = signals::IMU_CHANNELS[IMU_1.index()]
        .subscriber()
        .expect("too many subs on IMU_CHANNELS; increase SUBS");
    let mut mag_recv = signals::MAG_WATCH.anon_receiver();

    let mut fusion = fusion_instance();
    let mut selector = TimeoutSelector::new(TIMEOUT);
    let mut prev_ts: Option<Instant> = None;

    let orientation_sender = signals::ORIENTATION_WATCH.sender();
    let inertial_sender = signals::INERTIAL_WATCH.sender();

    loop {
        let sample: ImuSample =
            match select(sub0.next_message_pure(), sub1.next_message_pure()).await {
                Either::First(s) | Either::Second(s) => s,
            };

        if !selector.accept(sample.src, sample.ts) {
            continue;
        }

        let dt = prev_ts
            .map(|p| sample.ts.saturating_duration_since(p).as_micros() as f32 / 1e6)
            .unwrap_or(IMU_TARGET_DT);
        prev_ts = Some(sample.ts);

        let accel = sample.accel;
        let gyro = sample.gyro;

        let gyr_vec = FusionVector::new(gyro.x, gyro.y, gyro.z);
        let acc_vec = FusionVector::new(accel.x, accel.y, accel.z);

        let mag = mag_recv.try_get().filter(|m| m.ts.elapsed() <= MAG_FRESH);

        match mag {
            Some(MagSample { x, y, z, .. }) => {
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

        let out = Inertial {
            ts: sample.ts,
            body_accel_x: body_accel.x,
            body_accel_y: body_accel.y,
            body_accel_z: body_accel.z,
            body_gyro_x: body_angular_vel.x,
            body_gyro_y: body_angular_vel.y,
            body_gyro_z: body_angular_vel.z,
            ned_accel_north: inertial_accel_comp.x,
            ned_accel_east: inertial_accel_comp.y,
            ned_accel_down: inertial_accel_comp.z,
            ned_gyro_north: inertial_gyro.x,
            ned_gyro_east: inertial_gyro.y,
            ned_gyro_down: inertial_gyro.z,
        };

        orientation_sender.send(Orientation {
            ts: sample.ts,
            q: orientation,
        });
        inertial_sender.send(out);
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
