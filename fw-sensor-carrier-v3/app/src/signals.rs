//! Inter-task signals.
//!
//! One `PubSubChannel` per sensor kind carries every instance's readings, raw
//! and calibrated; each names its source in `src`. Publishing never waits: a
//! subscriber that falls behind, such as the SD writer during a slow card
//! write, loses its oldest readings and is told how many. The `Watch` carries
//! the latest state estimate to CAN.

use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;

use crate::types::{
    BaroReading, DhtReading, GnssReading, ImuReading, MagReading, Mark, SefLogSample, StateEstimate,
};

// Readouts publish from the interrupt executor, so the sample channels need
// a critical-section mutex.
macro_rules! define_sample_channel {
    ($channel:ident, $submit:ident: $T:ty, cap = $cap:expr, subs = $subs:expr) => {
        pub static $channel: PubSubChannel<CriticalSectionRawMutex, $T, $cap, $subs, 1> =
            PubSubChannel::new();

        pub fn $submit(sample: $T) {
            $channel.immediate_publisher().publish_immediate(sample);
        }
    };
}

define_sample_channel!(IMU_CHANNEL, submit_imu: ImuReading, cap = 512, subs = 2);
define_sample_channel!(MAG_CHANNEL, submit_mag: MagReading, cap = 32, subs = 2);
define_sample_channel!(GNSS_CHANNEL, submit_gnss: GnssReading, cap = 32, subs = 2);
define_sample_channel!(BARO_CHANNEL, submit_baro: BaroReading, cap = 64, subs = 2);
define_sample_channel!(DHT_CHANNEL, submit_dht: DhtReading, cap = 8, subs = 1);
// Per-chain estimator state for the SD log.
define_sample_channel!(STATE_CHANNEL, submit_state: SefLogSample, cap = 64, subs = 1);
define_sample_channel!(MARK_CHANNEL, submit_mark: Mark, cap = 4, subs = 1);

// Estimator and CAN both run on the thread-mode executor.
pub static STATE_ESTIMATE_WATCH: Watch<ThreadModeRawMutex, StateEstimate, 1> = Watch::new();
