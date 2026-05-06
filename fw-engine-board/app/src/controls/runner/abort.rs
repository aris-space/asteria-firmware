use crate::controls::abort_sequence::ABORT_SEQUENCE;
use crate::controls::actions::Actions;
use crate::controls::actions::actuate::actuate_onboard_valve;
use crate::controls::actions::wait::wait_no_abort;
use crate::controls::runner::{ABORT_INITIATION, FIRING_INFO, FiringInfo};
use crate::globals::STATE;
use embassy_time::{Duration, Timer};
use embedded_utils::fmt::warn;
use embedded_utils::info;

const ABORT_REQUEST_INTERVAL: Duration = Duration::from_millis(5000);
const ABORT_UPDATE_RATE: Duration = Duration::from_millis(50);

#[embassy_executor::task]
pub async fn abort_task_runner() {
    let mut abort_initiation_receiver = ABORT_INITIATION.receiver().unwrap();
    let firing_info_sender = FIRING_INFO.immediate_publisher();

    loop {
        if abort_initiation_receiver.try_changed().is_some() {
            warn!("[ABORT] INITIATED");
            firing_info_sender.publish_immediate(FiringInfo::FiringAborted);
            STATE.firing_aborted.sender().send(true);

            for action in &ABORT_SEQUENCE {
                info!("[ABORT] Executing action: {:?}", action);
                match action {
                    Actions::Wait(duration) => {
                        wait_no_abort(duration).await;
                    }
                    Actions::ActuateOnboard(valve) => {
                        actuate_onboard_valve(*valve).await;
                    }
                    _ => {
                        // Rest of the actions not needed in abort sequence
                        warn!("Unhandled abort action received: {:?}", action);
                    }
                };
            }

            info!("[ABORT] COMPLETED");
            Timer::after(ABORT_REQUEST_INTERVAL).await;

            // After completing the abort sequence, clear the abort initiation signal
            // making sure that the firing is not re-triggered unintentionally if it was sent again during the sequence
            let _ = abort_initiation_receiver.try_changed();
        }

        Timer::after(ABORT_UPDATE_RATE).await;
    }
}
