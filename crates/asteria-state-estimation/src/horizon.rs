use crate::{
    BarometerAided, BarometerInput, GnssAided, GnssInput, ImuAided, ImuInput, MagnetometerAided,
    MagnetometerInput, StateEstimator, UpdateError,
};
use core::cmp::Ordering;
use heapless::binary_heap::Max;

/*

delayed time horizon fusion.

We maintain a priority queue of thignies to be merged.

fusion timestamp for each sensor.
we run the fusion algo in order.
when sensor update arrives, we can in, we can put int into the queue to be fused.


 */

struct TimeHorizonConfig {
    max_span_us: usize,
}

// todo heap could possibly later be replaced with splay tree?
struct BufferedTimeHorizon<S: StateEstimator, const MAX_EVENTS: usize> {
    inner: S,
    queue: heapless::BinaryHeap<TimestampedEvent, Max, MAX_EVENTS>,
    config: TimeHorizonConfig,
}

impl<S: StateEstimator, const MAX_EVENTS: usize> BufferedTimeHorizon<S, MAX_EVENTS> {
    // this pushes the new event, and (possibly) fuses the old measurement if needed.
    fn push_event(&mut self, sensor_event: SensorEvent) {
        let s = self.queue.capacity();
    }

    fn fuse_event(&mut self, sensor_event: SensorEvent) {
        //snesor event is the newest produced by the queue that we hnow have to merge
    }
}

impl<S: StateEstimator, const MAX_EVENTS: usize> ImuAided for BufferedTimeHorizon<S, MAX_EVENTS> {
    fn update_imu(&mut self, input: ImuInput) -> Result<(), UpdateError> {
        // only if queue full or span is larger than configured span,
        // do we acutaly update the filter.
        self.push_event(input.into());
        Ok(()) //Todo
    }
}

impl<S: StateEstimator, const MAX_EVENTS: usize> BarometerAided
    for BufferedTimeHorizon<S, MAX_EVENTS>
{
    fn update_barometer(&mut self, input: BarometerInput) -> Result<(), UpdateError> {
        self.push_event(input.into());
        Ok(()) //Todo
    }
}

impl<S: StateEstimator, const MAX_EVENTS: usize> GnssAided for BufferedTimeHorizon<S, MAX_EVENTS> {
    fn update_gnss(&mut self, input: GnssInput) -> Result<(), UpdateError> {
        self.push_event(input.into());
        Ok(()) //Todo
    }
}

impl<S: StateEstimator, const MAX_EVENTS: usize> MagnetometerAided
    for BufferedTimeHorizon<S, MAX_EVENTS>
{
    fn update_magnetometer(&mut self, input: MagnetometerInput) -> Result<(), UpdateError> {
        self.push_event(input.into());
        Ok(()) //Todo
    }
}

struct TimestampedEvent {
    timestamp_us: u64,
    event: SensorEvent,
}

impl PartialEq<Self> for TimestampedEvent {
    fn eq(&self, other: &Self) -> bool {
        self.timestamp_us == other.timestamp_us
    }
}

impl Eq for TimestampedEvent {}

impl Ord for TimestampedEvent {
    fn cmp(&self, other: &Self) -> Ordering {
        self.timestamp_us.cmp(&other.timestamp_us)
    }
}

impl PartialOrd for TimestampedEvent {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        if self.timestamp_us < other.timestamp_us {
            Some(Ordering::Less)
        } else if self.timestamp_us > other.timestamp_us {
            Some(Ordering::Greater)
        } else {
            Some(Ordering::Equal)
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
enum SensorEvent {
    ImuUpdate(ImuInput),
    BaroUpdate(BarometerInput),
    MagUpdate(MagnetometerInput),
    GnssUpdate(GnssInput),
}

impl From<ImuInput> for SensorEvent {
    fn from(value: ImuInput) -> Self {
        Self::ImuUpdate(value)
    }
}
impl From<GnssInput> for SensorEvent {
    fn from(value: GnssInput) -> Self {
        Self::GnssUpdate(value)
    }
}
impl From<MagnetometerInput> for SensorEvent {
    fn from(value: MagnetometerInput) -> Self {
        Self::MagUpdate(value)
    }
}
impl From<BarometerInput> for SensorEvent {
    fn from(value: BarometerInput) -> Self {
        Self::BaroUpdate(value)
    }
}
