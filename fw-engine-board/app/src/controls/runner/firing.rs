#![allow(unused_assignments)]

use crate::controls::actions::actuate::actuate_onboard_valve;
use crate::controls::actions::detect::detect_with_abort;
use crate::controls::actions::wait::wait_with_abort;
use crate::controls::actions::watch::{WATCH_STATE, WatchState};
use crate::controls::actions::{ActionCompleteness, Actions};
use crate::controls::firing_sequence::FIRING_SEQUENCE;
use crate::controls::runner::{ABORT_INITIATION, FIRING_INFO, FIRING_INITIATION, FiringInfo};
use crate::globals::STATE;
use crate::sensors::Sensor;
use embassy_time::{Duration, Timer};
use embedded_utils::{fmt::warn, info};

const FIRING_REQUEST_INTERVAL: Duration = Duration::from_millis(5000);
const FIRING_UPDATE_RATE: Duration = Duration::from_millis(100);

#[embassy_executor::task]
pub async fn firing_task_runner() {
    let mut firing_initiation_receiver = FIRING_INITIATION.receiver().unwrap();
    let mut abort_initiation_receiver = ABORT_INITIATION.receiver().unwrap();

    let abort_initiator = ABORT_INITIATION.sender();
    let watch_state_sender = WATCH_STATE.sender();
    let firing_info_sender = FIRING_INFO.immediate_publisher();

    loop {
        // Check for firing initiation message
        if firing_initiation_receiver.try_changed().is_some() {
            // Firing Active, starting sequence
            info!("[FIRING] STARTED");

            firing_info_sender.publish_immediate(FiringInfo::FiringInitiated);
            STATE.firing_initiated.sender().send(true);

            // Execute the firing sequence
            for action in &FIRING_SEQUENCE {
                // Check for abort signal before each action
                if abort_initiation_receiver.try_changed().is_some() {
                    warn!("[FIRING] ABORTED");
                    break;
                }
                info!("[FIRING] Executing action: {:?}", action);

                match action {
                    Actions::Wait(duration) => {
                        // Wait with abort capability
                        let action_completeness =
                            wait_with_abort(duration, &mut abort_initiation_receiver).await;
                        // If the wait was aborted, exit the firing sequence by setting state to Idle
                        if action_completeness == ActionCompleteness::Failed {
                            break;
                        }
                    }
                    Actions::ActuateOnboard(valve) => {
                        actuate_onboard_valve(*valve).await;
                    }
                    Actions::Detect(detection) => {
                        // Perform detection with abort capability
                        let action_completeness = detect_with_abort(
                            detection,
                            &mut abort_initiation_receiver,
                            &abort_initiator,
                        )
                        .await;
                        // If the detection was aborted or failed, exit the firing sequence by setting state to Idle
                        if action_completeness == ActionCompleteness::Failed {
                            break;
                        }

                        // Publish the appropriate firing info based on the sensor type
                        match detection.sensor {
                            Sensor::EngineP(_) => {
                                firing_info_sender
                                    .publish_immediate(FiringInfo::CombustionDetected);
                                STATE.combustion_detected.sender().send(true);
                            }
                        }
                    }
                    Actions::Watch(watcher) => {
                        // Set the watch state to monitor the specified sensor
                        watch_state_sender.send(WatchState::Watch(watcher.clone()));
                    }
                    Actions::Ignore(_) => {
                        // Set the watch state to ignore sensor monitoring
                        watch_state_sender.send(WatchState::Ignore);
                    }
                };
            }
            // Reset the watch state to ignore after firing sequence completion
            watch_state_sender.send(WatchState::Ignore);

            info!("[FIRING] COMPLETED");

            firing_info_sender.publish_immediate(FiringInfo::FiringCompleted);
            STATE.firing_completed.sender().send(true);

            // Wait for a short duration to allow any late firing initiation signals to be ignored
            Timer::after(FIRING_REQUEST_INTERVAL).await;

            // After completing the firing sequence, mark the firing initiation signal,
            // making sure that the firing is not re-triggered unintentionally if it was sent again during the sequence
            let _ = firing_initiation_receiver.try_changed();
            let _ = abort_initiation_receiver.try_changed();
        }

        // Make sure abort queue is always read during Idle, no action has to be taken here
        // because the abort task is handling aborts
        let _ = abort_initiation_receiver.try_changed();

        Timer::after(FIRING_UPDATE_RATE).await;
    }
}
