use crate::controls::actions::Watcher;
use crate::controls::actions::detect::SENSOR_TIMEOUT;
use crate::controls::runner::ABORT_INITIATION;
use crate::drivers::{ENGINE_P_WATCH, IGNITER_P_WATCH};
use crate::sensors::Sensor;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Timer, with_timeout};
use hermes_can::messages::event_messages::FiringAbortInitiation;

#[derive(Clone, PartialEq)]
pub enum WatchState {
    Watch(Watcher),
    Ignore,
}

pub static WATCH_STATE: Watch<ThreadModeRawMutex, WatchState, 2> = Watch::new();
const WATCH_TIME_INTERVAL: Duration = Duration::from_millis(50);

#[embassy_executor::task]
pub async fn watch_task_runner() {
    let mut abort_receiver = ABORT_INITIATION.receiver().unwrap();
    let abort_initiator = ABORT_INITIATION.sender();

    let mut watch_state_receiver = WATCH_STATE.receiver().unwrap();
    let mut watch_state = WatchState::Ignore;

    loop {
        // Update the watch state from incoming messages
        if let Some(state) = watch_state_receiver.try_changed() {
            watch_state = state;
        }

        // If an abort has been initiated, switch to Ignore state
        if abort_receiver.try_changed().is_some() {
            watch_state = WatchState::Ignore;
        };

        // If we are in Watch state, monitor the sensor
        if let WatchState::Watch(watcher) = &watch_state {
            // Get the appropriate sensor receiver and threshold
            // These unwraps are safe because the total amount of receiver is smaller than the CAP
            let (mut sensor_receiver, threshold) = match watcher.sensor {
                Sensor::EngineP(threshold) => (ENGINE_P_WATCH.receiver().unwrap(), threshold),
                Sensor::IgniterP(threshold) => (IGNITER_P_WATCH.receiver().unwrap(), threshold),
            };

            // Try receiving a sensor update with a timeout
            if let Ok(value) = with_timeout(SENSOR_TIMEOUT, sensor_receiver.changed()).await {
                // Check if the sensor value meets the abort criteria
                // Increasing: value >= threshold
                // Decreasing: value <= threshold
                if (watcher.increasing && (value >= threshold))
                    || (!watcher.increasing && (value <= threshold))
                {
                    // Trigger abort if threshold is crossed
                    abort_initiator.send(FiringAbortInitiation);
                    // Switch to Ignore state after abort
                    watch_state = WatchState::Ignore;
                }
            } else {
                // If a timeout occurs, trigger abort
                abort_initiator.send(FiringAbortInitiation);

                // Switch to Ignore state after abort
                watch_state = WatchState::Ignore;
            }
        }

        // Make sure that no aborts are read if the Watcher is passive (on Ignore)
        if watch_state == WatchState::Ignore {
            let _ = abort_receiver.try_changed();
        }
        Timer::after(WATCH_TIME_INTERVAL).await;
    }
}
