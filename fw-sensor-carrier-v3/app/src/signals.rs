//! Inter-task signals.
//!
//! Per-sensor `PubSubChannel`s carry raw readout samples (one channel per
//! sensor instance, indexed by the sensor's id). Global `Watch`es carry
//! fused/derived outputs published by the processing tasks; the CAN task is
//! the single receiver for each.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;

use crate::measurements::{
    EnvSample, Environment, GnssSample, ImuSample, Inertial, MagSample, Orientation, Position,
    Pressure, PressureSample, RawMagSample, Velocity,
};
use crate::sensors::{BAROMETER_COUNT, DHT_COUNT, GNSS_COUNT, IMU_COUNT, MAGNETOMETER_COUNT};

macro_rules! define_sample_channels {
    (
        $channels:ident, $submit:ident, $submit_batch:ident :
        $T:ty, cap = $cap:expr, subs = $subs:expr, count = $count:expr
    ) => {
        pub static $channels: [PubSubChannel<CriticalSectionRawMutex, $T, $cap, $subs, 1>; $count] =
            [const { PubSubChannel::new() }; $count];

        #[allow(dead_code)]
        pub fn $submit(sample: $T) {
            $channels[sample.src.index()]
                .immediate_publisher()
                .publish_immediate(sample);
        }

        #[allow(dead_code)]
        pub fn $submit_batch(samples: &[$T]) {
            let Some(last) = samples.last() else { return };
            let publisher = $channels[last.src.index()].immediate_publisher();
            for sample in samples {
                publisher.publish_immediate(*sample);
            }
        }
    };
}

define_sample_channels!(IMU_CHANNELS, submit_imu_sample, submit_imu_sample_batch:
    ImuSample, cap = 64, subs = 1, count = IMU_COUNT);

define_sample_channels!(PRESSURE_CHANNELS, submit_pressure_sample, submit_pressure_sample_batch:
    PressureSample, cap = 16, subs = 2, count = BAROMETER_COUNT);

define_sample_channels!(MAG_CHANNELS, submit_mag_sample, submit_mag_sample_batch:
    RawMagSample, cap = 16, subs = 1, count = MAGNETOMETER_COUNT);

define_sample_channels!(GNSS_CHANNELS, submit_gnss_sample, submit_gnss_sample_batch:
    GnssSample, cap = 8, subs = 1, count = GNSS_COUNT);

define_sample_channels!(ENV_CHANNELS, submit_env_sample, submit_env_sample_batch:
    EnvSample, cap = 8, subs = 1, count = DHT_COUNT);

pub static PRESSURE_WATCH: Watch<CriticalSectionRawMutex, Pressure, 1> = Watch::new();
pub static ENVIRONMENT_WATCH: Watch<CriticalSectionRawMutex, Environment, 1> = Watch::new();
pub static ORIENTATION_WATCH: Watch<CriticalSectionRawMutex, Orientation, 1> = Watch::new();
pub static INERTIAL_WATCH: Watch<CriticalSectionRawMutex, Inertial, 1> = Watch::new();
pub static MAG_WATCH: Watch<CriticalSectionRawMutex, MagSample, 1> = Watch::new();
pub static POSITION_WATCH: Watch<CriticalSectionRawMutex, Position, 1> = Watch::new();
pub static VELOCITY_WATCH: Watch<CriticalSectionRawMutex, Velocity, 1> = Watch::new();
