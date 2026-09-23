//! Timestamp-ordered delivery of the eight sensor streams to SEF-light.

use defmt::info;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer};

use super::{EVENT_HOLDBACK, Event, IMU_DIAGNOSTIC_PERIOD, Processor};
use crate::sensors::{BARO_BUS_1, BARO_BUS_2, GNSS_0, GNSS_1, IMU_0, IMU_1, MAG_BUS_1, MAG_BUS_2};
use crate::signals;

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut imu0 = signals::IMU_CHANNELS[IMU_0.index()]
        .subscriber()
        .expect("SEF IMU 0 subscriber");
    let mut imu1 = signals::IMU_CHANNELS[IMU_1.index()]
        .subscriber()
        .expect("SEF IMU 1 subscriber");
    let mut baro0 = signals::BARO_CHANNELS[BARO_BUS_1.index()]
        .subscriber()
        .expect("SEF barometer 0 subscriber");
    let mut baro1 = signals::BARO_CHANNELS[BARO_BUS_2.index()]
        .subscriber()
        .expect("SEF barometer 1 subscriber");
    let mut gnss0 = signals::GNSS_CHANNELS[GNSS_0.index()]
        .subscriber()
        .expect("SEF GNSS 0 subscriber");
    let mut gnss1 = signals::GNSS_CHANNELS[GNSS_1.index()]
        .subscriber()
        .expect("SEF GNSS 1 subscriber");
    let mut mag0 = signals::MAG_CHANNELS[MAG_BUS_1.index()]
        .subscriber()
        .expect("SEF magnetometer 0 subscriber");
    let mut mag1 = signals::MAG_CHANNELS[MAG_BUS_2.index()]
        .subscriber()
        .expect("SEF magnetometer 1 subscriber");
    let mut processor = Processor::new().expect("SEF-light configuration must be valid");
    // Keep the oldest unread sample from each source. FIFO batches are published separately;
    // the short holdback lets their older timestamps arrive before newer aiding.
    let mut heads: [Option<Event>; 8] = [None; 8];
    let mut dropped_imu = [0_u64; 2];
    let mut dropped_aiding = [0_u64; 6];
    let mut late_events = [0_u32; 2];
    let mut last_event_time: Option<Instant> = None;
    let mut max_imu_backlog = [0_u64; 2];
    let mut queue_report_at = Instant::now() + IMU_DIAGNOSTIC_PERIOD;

    loop {
        max_imu_backlog[0] = max_imu_backlog[0].max(imu0.available());
        max_imu_backlog[1] = max_imu_backlog[1].max(imu1.available());

        if heads[0].is_none() {
            match imu0.try_next_message() {
                Some(WaitResult::Message(sample)) => heads[0] = Some(Event::Imu(sample)),
                Some(WaitResult::Lagged(count)) => dropped_imu[0] += count,
                None => {}
            }
        }
        if heads[1].is_none() {
            match imu1.try_next_message() {
                Some(WaitResult::Message(sample)) => heads[1] = Some(Event::Imu(sample)),
                Some(WaitResult::Lagged(count)) => dropped_imu[1] += count,
                None => {}
            }
        }
        if heads[2].is_none() {
            match baro0.try_next_message() {
                Some(WaitResult::Message(sample)) => heads[2] = Some(Event::Barometer(sample)),
                Some(WaitResult::Lagged(count)) => dropped_aiding[0] += count,
                None => {}
            }
        }
        if heads[3].is_none() {
            match baro1.try_next_message() {
                Some(WaitResult::Message(sample)) => heads[3] = Some(Event::Barometer(sample)),
                Some(WaitResult::Lagged(count)) => dropped_aiding[1] += count,
                None => {}
            }
        }
        if heads[4].is_none() {
            match gnss0.try_next_message() {
                Some(WaitResult::Message(sample)) => heads[4] = Some(Event::Gnss(sample)),
                Some(WaitResult::Lagged(count)) => dropped_aiding[2] += count,
                None => {}
            }
        }
        if heads[5].is_none() {
            match gnss1.try_next_message() {
                Some(WaitResult::Message(sample)) => heads[5] = Some(Event::Gnss(sample)),
                Some(WaitResult::Lagged(count)) => dropped_aiding[3] += count,
                None => {}
            }
        }
        if heads[6].is_none() {
            match mag0.try_next_message() {
                Some(WaitResult::Message(sample)) => heads[6] = Some(Event::Magnetometer(sample)),
                Some(WaitResult::Lagged(count)) => dropped_aiding[4] += count,
                None => {}
            }
        }
        if heads[7].is_none() {
            match mag1.try_next_message() {
                Some(WaitResult::Message(sample)) => heads[7] = Some(Event::Magnetometer(sample)),
                Some(WaitResult::Lagged(count)) => dropped_aiding[5] += count,
                None => {}
            }
        }

        // Feed all eight streams in timestamp order. In-order aiding avoids a
        // full SEF history replay at each 40 Hz barometer observation.
        let next = heads
            .iter()
            .enumerate()
            .filter_map(|(index, event)| event.map(|event| (index, event.ts())))
            .min_by_key(|&(_, ts)| ts);
        if let Some((index, ts)) = next {
            if Instant::now().saturating_duration_since(ts) >= EVENT_HOLDBACK {
                let event = heads[index].take().expect("selected stream head");
                if last_event_time.is_some_and(|last| ts < last) {
                    late_events[usize::from(index >= 2)] += 1;
                } else {
                    last_event_time = Some(ts);
                    processor.handle(event);
                }
            } else {
                Timer::at(ts + EVENT_HOLDBACK).await;
            }
        } else {
            Timer::after(Duration::from_millis(2)).await;
        }

        if Instant::now() >= queue_report_at {
            info!(
                "SEF queues/10s: imu_dropped=[{},{}], aiding_dropped=[{},{},{},{},{},{}], late=[{},{}], max_imu_backlog=[{},{}]",
                dropped_imu[0],
                dropped_imu[1],
                dropped_aiding[0],
                dropped_aiding[1],
                dropped_aiding[2],
                dropped_aiding[3],
                dropped_aiding[4],
                dropped_aiding[5],
                late_events[0],
                late_events[1],
                max_imu_backlog[0],
                max_imu_backlog[1]
            );
            dropped_imu = [0; 2];
            dropped_aiding = [0; 6];
            late_events = [0; 2];
            max_imu_backlog = [0; 2];
            queue_report_at = Instant::now() + IMU_DIAGNOSTIC_PERIOD;
        }
    }
}
