//! Inter-task signals.
//!
//! Per-sensor `PubSubChannel`s carry raw readout samples (one channel per
//! sensor instance, indexed by the sensor's id). The global `Watch` carries
//! the state estimate to its output task.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;

use crate::sensors::{BAROMETER_COUNT, DHT_COUNT, GNSS_COUNT, IMU_COUNT, MAGNETOMETER_COUNT};
use crate::types::{
    BaroSample, DhtSample, GnssSample, ImuSample, MagSample, RawMagSample, VerticalEstimate,
};

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

define_sample_channels!(BARO_CHANNELS, submit_baro_sample, submit_baro_sample_batch:
    BaroSample, cap = 16, subs = 1, count = BAROMETER_COUNT);

define_sample_channels!(MAG_CHANNELS, submit_mag_sample, submit_mag_sample_batch:
    MagSample, cap = 16, subs = 1, count = MAGNETOMETER_COUNT);

// Raw, pre-calibration mag samples for the calibration task to consume
// while the device is being tumbled. Off the hot path otherwise.
define_sample_channels!(RAW_MAG_CHANNELS, submit_raw_mag_sample, submit_raw_mag_sample_batch:
    RawMagSample, cap = 16, subs = 1, count = MAGNETOMETER_COUNT);

define_sample_channels!(GNSS_CHANNELS, submit_gnss_sample, submit_gnss_sample_batch:
    GnssSample, cap = 8, subs = 1, count = GNSS_COUNT);

define_sample_channels!(DHT_CHANNELS, submit_dht_sample, submit_dht_sample_batch:
    DhtSample, cap = 8, subs = 1, count = DHT_COUNT);

pub static VERTICAL_ESTIMATE_WATCH: Watch<CriticalSectionRawMutex, VerticalEstimate, 1> =
    Watch::new();
