use crate::controls::actions::{ActionCompleteness, Detection};
use crate::drivers::{ENGINE_P_WATCH, IGNITER_P_WATCH, WATCH};
use crate::sensors::Sensor;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Receiver, Sender};
use embassy_time::{Duration, Instant, with_timeout};
use embedded_utils::fmt::warn;
use hermes_can::messages::event_messages::FiringAbortInitiation;

// Timeout for waiting on sensor changes
pub(crate) const SENSOR_TIMEOUT: Duration = Duration::from_millis(250);

// Duration the sensor must remain in the target state to confirm detection
const DETECTION_DURATION: Duration = Duration::from_millis(50);

#[derive(PartialEq)]
enum DetectionState {
    Running,
    Idle,
}

pub async fn detect_with_abort<'a>(
    detection: &Detection,
    abort_initiation_receiver: &mut Receiver<
        'a,
        CriticalSectionRawMutex,
        FiringAbortInitiation,
        WATCH,
    >,
    abort_initiator: &Sender<'a, CriticalSectionRawMutex, FiringAbortInitiation, WATCH>,
) -> ActionCompleteness {
    let (mut sensor_receiver, target) = match detection.sensor {
        Sensor::EngineP(target) => {
            (ENGINE_P_WATCH.receiver()
                .expect("Engine Pressure Watch receiver not available, should not happen, increase WATCH count"),
             target)
        }
        Sensor::IgniterP(target) => {
            (IGNITER_P_WATCH.receiver()
                .expect("Igniter Pressure Watch receiver not available, should not happen, increase WATCH count"),
             target)
        }
    };

    // Start the detection process with a timeout for the entire patience duration
    with_timeout(detection.patience, async {
        let mut state = DetectionState::Idle;
        let mut detection_start = Instant::now();
        loop {
            if abort_initiation_receiver.try_changed().is_some() {
                return ActionCompleteness::Failed;
            }

            // Try receiving a sensor update with a timeout
            if let Ok(value) = with_timeout(SENSOR_TIMEOUT, sensor_receiver.changed()).await {
                // Check if the sensor value meets the detection criteria
                // Increasing: value >= target
                // Decreasing: value <= target
                if (detection.increasing && (value >= target))
                    || (!detection.increasing && (value <= target))
                {
                    // If we were idle, start the detection timer
                    if state == DetectionState::Idle {
                        state = DetectionState::Running;
                        detection_start = Instant::now();
                    }
                    // If we are already running, check if the detection duration has been met
                    else if detection_start.elapsed() >= DETECTION_DURATION {
                        return ActionCompleteness::Successful;
                    }
                }
                // Sensor value does not meet criteria, reset state to idle
                else {
                    state = DetectionState::Idle;
                }
            }
            // Sensor data not received within timeout, abort detection
            else {
                // Trigger abort if sensor timeout occurs
                abort_initiator.send(FiringAbortInitiation);
                warn!("[DETECTION] Sensor Timeout");
                return ActionCompleteness::Failed;
            }
        }
    })
    .await
    .unwrap_or_else(|_| {
        // Trigger abort if overall detection patience times out
        abort_initiator.send(FiringAbortInitiation);
        warn!("[DETECTION] Target Timeout");
        ActionCompleteness::Failed
    })
}
