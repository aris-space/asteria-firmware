//! Inter-task signals.
//!
//! Two flavours:
//! 1. Per-sensor (indexed by sensor id) PubSub + Watch for raw readouts.
//! 2. Single global Watch + PubSub for derived/fused outputs published by the
//!    processing tasks.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;
use hermes_can::messages::sensor_data::{
    EnvironmentalData, ImuData as CanImuData, PositionData, VelocityData,
};
use nalgebra::UnitQuaternion;

use crate::measurements::{EnvSample, GnssSample, ImuSample, MagSample, PressureSample};
use crate::sensors::{
    BAROMETER_COUNT, BarometerId, DHT_COUNT, DhtId, GNSS_COUNT, GnssId, IMU_COUNT, ImuId,
    MAGNETOMETER_COUNT, MagnetometerId,
};

macro_rules! define_signal {
    (
        $channels:ident, $watches:ident, $submit:ident, $submit_batch:ident, $watch_getter:ident :
        $T:ty, $id_ty:ty,
        cap = $cap:expr, subs = $subs:expr, pubs = $pubs:expr,
        count = $count:expr, watchers = $watchers:expr
    ) => {
        pub static $channels: [PubSubChannel<CriticalSectionRawMutex, $T, $cap, $subs, $pubs>;
            $count] = [const { PubSubChannel::new() }; $count];

        pub static $watches: [Watch<CriticalSectionRawMutex, $T, $watchers>; $count] =
            [const { Watch::new() }; $count];

        #[allow(dead_code)]
        pub fn $watch_getter(id: $id_ty) -> &'static Watch<CriticalSectionRawMutex, $T, $watchers> {
            &$watches[id.index()]
        }

        #[allow(dead_code)]
        pub fn $submit(sample: $T) {
            let idx = sample.sensor_id.index();
            $channels[idx]
                .immediate_publisher()
                .publish_immediate(sample);
            $watches[idx].sender().send(sample);
        }

        #[allow(dead_code)]
        pub fn $submit_batch(samples: &[$T]) {
            let Some(last) = samples.last() else { return };
            let idx = last.sensor_id.index();
            let publisher = $channels[idx].immediate_publisher();
            for sample in samples {
                publisher.publish_immediate(*sample);
            }
            $watches[idx].sender().send(*last);
        }
    };
}

define_signal!(
    IMU_CHANNELS, IMU_WATCHES, submit_imu_sample, submit_imu_samples, imu_watch:
    ImuSample, ImuId,
    cap = 64, subs = 4, pubs = 2,
    count = IMU_COUNT, watchers = 4
);

define_signal!(
    PRESSURE_CHANNELS, PRESSURE_WATCHES, submit_pressure_sample, submit_pressure_samples, pressure_watch:
    PressureSample, BarometerId,
    cap = 16, subs = 4, pubs = 2,
    count = BAROMETER_COUNT, watchers = 4
);

define_signal!(
    MAG_CHANNELS, MAG_WATCHES, submit_mag_sample, submit_mag_samples, mag_watch:
    MagSample, MagnetometerId,
    cap = 16, subs = 4, pubs = 2,
    count = MAGNETOMETER_COUNT, watchers = 4
);

define_signal!(
    GNSS_CHANNELS, GNSS_WATCHES, submit_gnss_sample, submit_gnss_samples, gnss_watch:
    GnssSample, GnssId,
    cap = 8, subs = 4, pubs = 2,
    count = GNSS_COUNT, watchers = 4
);

define_signal!(
    ENV_CHANNELS, ENV_WATCHES, submit_env_sample, submit_env_samples, env_watch:
    EnvSample, DhtId,
    cap = 8, subs = 4, pubs = 2,
    count = DHT_COUNT, watchers = 4
);

// --- Derived / fused outputs (published by tasks/processing). ----------------

/// Filtered pressure scalar (mbar), from the pressure processing task.
pub static PRESSURE_FUSED_WATCH: Watch<CriticalSectionRawMutex, f32, 4> = Watch::new();
pub static PRESSURE_FUSED_PUBSUB: PubSubChannel<CriticalSectionRawMutex, f32, 10, 4, 2> =
    PubSubChannel::new();

/// Fused environmental data (temperature, humidity, pressure).
pub static ENVIRONMENTAL_WATCH: Watch<CriticalSectionRawMutex, EnvironmentalData, 4> = Watch::new();

/// Fused orientation quaternion (body -> NED, true north).
pub static ORIENTATION_WATCH: Watch<CriticalSectionRawMutex, UnitQuaternion<f32>, 4> = Watch::new();

/// Derived inertial CAN frame (body + inertial accel/gyro).
pub static INERTIAL_WATCH: Watch<CriticalSectionRawMutex, CanImuData, 4> = Watch::new();

/// Calibrated magnetic field in nT.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagFieldNt {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub static MAG_FIELD_WATCH: Watch<CriticalSectionRawMutex, MagFieldNt, 4> = Watch::new();

/// Position + velocity derived from the best GNSS source.
pub static POSITION_WATCH: Watch<CriticalSectionRawMutex, PositionData, 4> = Watch::new();
pub static VELOCITY_WATCH: Watch<CriticalSectionRawMutex, VelocityData, 4> = Watch::new();
