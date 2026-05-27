use crate::filters::{Filter, MovingAverage};
use crate::sensors::barometer;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_sync::pubsub::{ImmediatePublisher, PubSubChannel};
use embassy_sync::watch::Sender;

pub const CAP: usize = 10;
pub const PUB: usize = 1;
pub const SUB: usize = 2;
pub const WATCH: usize = 3;

pub static PRESSURE_DRIVER_PUBSUB: PubSubChannel<
    ThreadModeRawMutex,
    PressureEstimate,
    CAP,
    SUB,
    PUB,
> = PubSubChannel::new();

pub static PRESSURE_DRIVER: OnceLock<PressureDriver> = OnceLock::new();
const MOVING_AVERAGE_COUNT: usize = barometer::SAMPLE_FREQUENCY_HZ.div_ceil(5) as usize;

pub type PressureDataRaw = f32;
pub type PressureEstimate = f32;

pub struct PressureDriver<'a> {
    /// Data shared between references to the `PressureDriver`.
    shared: Mutex<CriticalSectionRawMutex, MovingAverage<f32, MOVING_AVERAGE_COUNT>>,
    /// On this channel, we publish all pressure estimates. This may be used for lossless logging
    publisher: ImmediatePublisher<'a, ThreadModeRawMutex, PressureEstimate, CAP, SUB, PUB>,
    /// This channel is used to publish the latest pressure estimate, which can be used to wait
    /// for the newest value, ie. a polling interface.
    watch: Sender<'a, ThreadModeRawMutex, PressureEstimate, WATCH>,
}

impl<'a> PressureDriver<'a> {
    pub fn new(
        publisher: ImmediatePublisher<'a, ThreadModeRawMutex, PressureEstimate, CAP, SUB, PUB>,
        watch: Sender<'a, ThreadModeRawMutex, PressureEstimate, WATCH>,
    ) -> PressureDriver<'a> {
        Self {
            shared: Mutex::new(MovingAverage::new()),
            publisher,
            watch,
        }
    }

    pub async fn update_with_raw_data(&self, data: PressureDataRaw) {
        let estimate = {
            let mut shared = self.shared.lock().await;
            shared.update(data)
        };

        self.publisher.publish_immediate(estimate);
        self.watch.send(estimate);
    }
}
