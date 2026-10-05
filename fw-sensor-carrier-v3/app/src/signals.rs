//! Inter-task signals.
//!
//! One `PubSubChannel` per sensor kind carries the calibrated samples of every
//! instance; each sample names its source in `src`. The global `Watch` carries
//! the latest state estimate to CAN. A bounded queue carries full-rate samples
//! to the SD task without waiting for card I/O in any producer.

use core::sync::atomic::{AtomicU32, Ordering};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::channel::Channel;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;

use crate::types::{
    BaroSample, DhtSample, GnssSample, ImuSample, MagSample, RawMagSample, SdLogRecord,
    StateEstimate,
};

// Readouts publish from the interrupt executor, so the sample channels and
// the SD queue need a critical-section mutex.
macro_rules! define_sample_channel {
    ($channel:ident, $submit:ident: $T:ty, cap = $cap:expr, subs = $subs:expr) => {
        pub static $channel: PubSubChannel<CriticalSectionRawMutex, $T, $cap, $subs, 1> =
            PubSubChannel::new();

        pub fn $submit(sample: $T) {
            $channel.immediate_publisher().publish_immediate(sample);
        }
    };
}

define_sample_channel!(IMU_CHANNEL, submit_imu_sample: ImuSample, cap = 512, subs = 2);
define_sample_channel!(MAG_CHANNEL, submit_mag_sample: MagSample, cap = 32, subs = 1);
define_sample_channel!(RAW_MAG_CHANNEL, submit_raw_mag_sample: RawMagSample, cap = 32, subs = 1);
define_sample_channel!(GNSS_CHANNEL, submit_gnss_sample: GnssSample, cap = 16, subs = 1);
define_sample_channel!(BARO_CHANNEL, submit_baro_sample: BaroSample, cap = 32, subs = 1);
define_sample_channel!(DHT_CHANNEL, submit_dht_sample: DhtSample, cap = 16, subs = 1);

// Estimator and CAN both run on the thread-mode executor.
pub static STATE_ESTIMATE_WATCH: Watch<ThreadModeRawMutex, StateEstimate, 1> = Watch::new();

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
