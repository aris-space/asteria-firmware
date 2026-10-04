//! Timestamp-ordered delivery of the sensor streams to SEF-light.

use defmt::warn;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::{DynSubscriber, PubSubChannel, WaitResult};
use embassy_time::{Duration, Instant, Timer};

use super::{Event, Processor};
use crate::signals;

// FIFO batches are published separately; the holdback lets their older
// timestamps arrive before newer aiding samples.
const EVENT_HOLDBACK: Duration = Duration::from_millis(35);
const IDLE_POLL: Duration = Duration::from_millis(2);

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut imu = signals::IMU_CHANNELS.each_ref().map(subscribe);
    let mut baro = signals::BARO_CHANNELS.each_ref().map(subscribe);
    let mut gnss = signals::GNSS_CHANNELS.each_ref().map(subscribe);
    let mut mag = signals::MAG_CHANNELS.each_ref().map(subscribe);
    let mut processor = Processor::new().expect("SEF-light configuration must be valid");
    // The oldest unread sample from each stream.
    let mut heads: [Option<Event>; 8] = [None; 8];
    let mut last_event_time: Option<Instant> = None;

    loop {
        let (imu_heads, rest) = heads.split_at_mut(2);
        let (baro_heads, rest) = rest.split_at_mut(2);
        let (gnss_heads, mag_heads) = rest.split_at_mut(2);
        refill(imu_heads, &mut imu, Event::Imu, "IMU");
        refill(baro_heads, &mut baro, Event::Baro, "baro");
        refill(gnss_heads, &mut gnss, Event::Gnss, "GNSS");
        refill(mag_heads, &mut mag, Event::Mag, "mag");

        // Feed all streams in timestamp order. In-order aiding avoids a full
        // SEF history replay at each barometer observation.
        let Some(head) = heads
            .iter_mut()
            .filter(|head| head.is_some())
            .min_by_key(|head| head.map(Event::ts))
        else {
            Timer::after(IDLE_POLL).await;
            continue;
        };
        let ts = head.map(Event::ts).expect("filtered to filled heads");
        if Instant::now() < ts + EVENT_HOLDBACK {
            Timer::at(ts + EVENT_HOLDBACK).await;
            continue;
        }
        let event = head.take().expect("filtered to filled heads");
        if let Some(last) = last_event_time
            && ts < last
        {
            warn!(
                "SEF dropped a sample {} us older than the last",
                (last - ts).as_micros()
            );
            continue;
        }
        last_event_time = Some(ts);
        processor.handle(event);
    }
}

fn subscribe<T: Clone, const CAP: usize, const SUBS: usize, const PUBS: usize>(
    channel: &'static PubSubChannel<CriticalSectionRawMutex, T, CAP, SUBS, PUBS>,
) -> DynSubscriber<'static, T> {
    channel
        .dyn_subscriber()
        .expect("SEF subscriber slot must be free")
}

fn refill<T: Clone>(
    heads: &mut [Option<Event>],
    subscribers: &mut [DynSubscriber<'static, T>],
    event: fn(T) -> Event,
    stream: &str,
) {
    for (head, subscriber) in heads.iter_mut().zip(subscribers) {
        if head.is_some() {
            continue;
        }
        match subscriber.try_next_message() {
            Some(WaitResult::Message(sample)) => *head = Some(event(sample)),
            Some(WaitResult::Lagged(count)) => warn!("SEF dropped {} {} samples", count, stream),
            None => {}
        }
    }
}
