//! Inter-task signals.
//!
//! Per-sensor `PubSubChannel`s carry raw readout samples (one channel per
//! sensor instance, indexed by the sensor's id). The global `Watch` carries
//! the latest state estimate to CAN. A bounded queue carries full-rate samples
//! to the SD task without waiting for card I/O in any producer.

use core::sync::atomic::{AtomicU32, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;

use crate::sensors::{BAROMETER_COUNT, DHT_COUNT, GNSS_COUNT, IMU_COUNT, MAGNETOMETER_COUNT};
use crate::types::{
    BaroSample, DhtSample, GnssSample, ImuSample, MagSample, RawMagSample, SdLogRecord,
    StateEstimate,
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
    ImuSample, cap = 256, subs = 2, count = IMU_COUNT);

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

pub static STATE_ESTIMATE_WATCH: Watch<CriticalSectionRawMutex, StateEstimate, 1> = Watch::new();

// Readout and estimation never wait for SD writes. The writer reports any
// overflow so incomplete logs are visible during a bench run.
pub static SD_LOG_CHANNEL: Channel<CriticalSectionRawMutex, SdLogRecord, 512> = Channel::new();
pub static SD_LOG_DROPPED: [AtomicU32; SdLogRecord::KIND_COUNT] =
    [const { AtomicU32::new(0) }; SdLogRecord::KIND_COUNT];

pub fn submit_sd_log(record: SdLogRecord) {
    let kind = record.kind();
    if SD_LOG_CHANNEL.try_send(record).is_err() {
        SD_LOG_DROPPED[kind].fetch_add(1, Ordering::Relaxed);
    }
}
