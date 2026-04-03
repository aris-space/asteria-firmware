use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;

use crate::measurements::{ImuSample, PressureSample};
use crate::sensors::{BAROMETER_COUNT, BarometerId, IMU_COUNT, ImuId};

macro_rules! define_signal {
    (
        $channels:ident, $watches:ident, $submit:ident, $submit_batch:ident, $watch_getter:ident :
        $T:ty, $id_ty:ty,
        cap = $cap:expr, subs = $subs:expr, pubs = $pubs:expr,
        count = $count:expr, watchers = $watchers:expr
    ) => {
        pub static $channels: [PubSubChannel<CriticalSectionRawMutex, $T, $cap, $subs, $pubs>; $count] =
            [const { PubSubChannel::new() }; $count];

        pub static $watches: [Watch<CriticalSectionRawMutex, $T, $watchers>; $count] =
            [const { Watch::new() }; $count];

        pub fn $watch_getter(id: $id_ty) -> &'static Watch<CriticalSectionRawMutex, $T, $watchers> {
            &$watches[id.index()]
        }

        pub fn $submit(sample: $T) {
            let idx = sample.sensor_id.index();
            $channels[idx].immediate_publisher().publish_immediate(sample);
            $watches[idx].sender().send(sample);
        }

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
