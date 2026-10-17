use asteria_sef_core::NavigationState;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;

use crate::sensors::{GnssId, ImuId};
use crate::types::{
    BaroReading, DhtReading, GnssReading, ImuReading, MagReading, Mark, SdSession, SefLogSample,
};

macro_rules! define_sample_channel {
    ($channel:ident, $submit:ident: $T:ty, cap = $cap:expr, subs = $subs:expr) => {
        pub static $channel: PubSubChannel<CriticalSectionRawMutex, $T, $cap, $subs, 1> =
            PubSubChannel::new();

        pub fn $submit(sample: $T) {
            $channel.immediate_publisher().publish_immediate(sample);
        }
    };
}

define_sample_channel!(IMU_CHANNEL, submit_imu: ImuReading, cap = 512, subs = 4);
define_sample_channel!(MAG_CHANNEL, submit_mag: MagReading, cap = 32, subs = 4);
define_sample_channel!(GNSS_CHANNEL, submit_gnss: GnssReading, cap = 32, subs = 4);
define_sample_channel!(BARO_CHANNEL, submit_baro: BaroReading, cap = 64, subs = 4);
define_sample_channel!(DHT_CHANNEL, submit_dht: DhtReading, cap = 8, subs = 2);
define_sample_channel!(STATE_CHANNEL, submit_state: SefLogSample, cap = 64, subs = 3);
define_sample_channel!(MARK_CHANNEL, submit_mark: Mark, cap = 4, subs = 1);

pub static STATE_ESTIMATE: Watch<CriticalSectionRawMutex, NavigationState, 5> = Watch::new();
pub static SELECTED_IMU: Watch<CriticalSectionRawMutex, ImuId, 1> = Watch::new();
pub static SELECTED_GNSS: Watch<CriticalSectionRawMutex, GnssId, 1> = Watch::new();
pub static SELECTED_BARO: Watch<CriticalSectionRawMutex, BaroReading, 2> = Watch::new();
pub static SELECTED_MAG: Watch<CriticalSectionRawMutex, MagReading, 1> = Watch::new();
pub static SELECTED_DHT: Watch<CriticalSectionRawMutex, DhtReading, 1> = Watch::new();
pub static SD_LOGGING: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();
pub static SD_SESSION: Watch<CriticalSectionRawMutex, SdSession, 1> = Watch::new();
pub static STARTUP_COMPLETE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
pub static USB_LINK_SIGNAL: Signal<CriticalSectionRawMutex, bool> = Signal::new();
