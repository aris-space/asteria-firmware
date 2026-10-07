//! CAN message sending and storage.

use can_utils::broadcast::Broadcast;
use can_utils::collector::Collector;
use data_core::can::{hal::CanDecode, sparse_decodable_can_message};
use datatypes::status::BuildInformationCommon;
use dp_recovery_board::{ActuatorStatus, DeploymentState, SteeringPositions};
use embassy_stm32::can::CanTx;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Watch;
use embassy_time::Instant;

#[cfg(not(feature = "defmt"))]
use crate::CAN_TX_TIMEOUT;
use crate::actuator_control::{
    ARMING_STATE, DEPLOYMENT_OCCURRED, DEPLOYMENT_SERVO_STATUS, SEPARATION_OCCURRED,
    SEPARATION_SERVO_STATUS, STEERING_STATUS, SteeringStatuses, WATCHDOG_STATE,
};
use crate::{CAN_TX_TIMEOUT, STATUS_CREATION_INTERVAL};
use can_utils::rxtx::TypedCanTransmit;
use datatypes::status::{ArmingState, StatusCommonMessage};
use dp_recovery_board::{RecoveryBoardStatus, WatchdogState};
use embassy_futures::join::join3;
use embassy_time::{Timer, with_timeout};
use embedded_utils::fmt::*;

// This enum encompases all messages that the Recovery board needs to receive.
sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
        UTCTimeUpdate(dp_system_management::Message::UTCTimeUpdate),

        SteeringTargetPositions(dp_recovery_board::Message::SteeringTargetPositions),
        RecoveryPowerConfig(dp_recovery_board::Message::RecoveryPowerConfig),

        SeparationTrigger(dp_recovery_board::Message::SeparationTrigger),
        DeploymentTrigger(dp_recovery_board::Message::DeploymentTrigger),
    }
}

// This will fail to compile if the number of enabled ids exceeds the number of filters
const __ASSERT_LEN_OK: () = {
    const FILTER_COUNT: usize = 28;
    if ReceivedMessage::SUPPORTED_IDS.len() >= FILTER_COUNT - 1 {
        core::panic!("Too many receiving can ids");
    }
};

/// Values received over CAN, handed to the tasks that consume them.
#[derive(Collector)]
#[collector(
    message_type = "ReceivedMessage",
    update_expr = "#field.sender().send(#value);"
)]
pub struct Inputs {
    /// watch for giving steering target positions to steering_task
    #[collector(pattern = "ReceivedMessage::SteeringTargetPositions(#value)")]
    pub steering_target_positions: Watch<CriticalSectionRawMutex, SteeringPositions, 3>,

    /// watch for setting steering power
    pub steering_power: Watch<CriticalSectionRawMutex, bool, 1>,
}

pub static INPUTS: Inputs = Inputs {
    steering_target_positions: Watch::new(),
    steering_power: Watch::new(),
};

#[derive(Broadcast)]
#[broadcast(loop_type = "can_utils::broadcast::ResponsiveLoop")]
pub struct Outputs {
    /// Position data read from the motors
    #[broadcast(
        filter_map = "#value.map(dp_recovery_board::Message::SteeringActualPositions)",
        min_freq_hz = 0.1,
        max_freq_hz = 15.
    )]
    pub steering_actual_positions: Watch<ThreadModeRawMutex, Option<SteeringPositions>, 2>,
    #[broadcast(
        map = "dp_recovery_board::Message::BuildInfo(#value)",
        min_freq_hz = 0.2,
        max_freq_hz = 0.2
    )]
    pub build_info: Watch<ThreadModeRawMutex, BuildInformationCommon, 1>,
    /// Whether the CATS backup computer has triggered separation (CATS IO 1)
    #[broadcast(
        map = "dp_recovery_board::Message::CATSSeparationState(#value)",
        min_freq_hz = 0.5,
        max_freq_hz = 12.0
    )]
    pub cats_separation: Watch<CriticalSectionRawMutex, DeploymentState, 1>,
    /// Whether the CATS backup computer has triggered main deployment (CATS IO 2)
    #[broadcast(
        map = "dp_recovery_board::Message::CATSDeploymentState(#value)",
        min_freq_hz = 0.5,
        max_freq_hz = 12.0
    )]
    pub cats_deployment: Watch<CriticalSectionRawMutex, DeploymentState, 1>,
}

pub static OUTPUTS: Outputs = Outputs {
    steering_actual_positions: Watch::new(),
    build_info: Watch::new(),
    cats_separation: Watch::new(),
    cats_deployment: Watch::new(),
};

#[embassy_executor::task]
pub async fn can_tx_task(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
    let status_creation_task = async {
        let mut last = Instant::now();
        let mut steering_status_rx = STEERING_STATUS.receiver().unwrap();
        let mut separation_status_rx = SEPARATION_SERVO_STATUS.receiver().unwrap();
        let mut deployment_status_rx = DEPLOYMENT_SERVO_STATUS.receiver().unwrap();
        let mut steering_watchdog_status_rx = WATCHDOG_STATE.receiver().unwrap();
        let mut arming_state_rx = ARMING_STATE.receiver().unwrap();

        let mut sep1_status = ActuatorStatus::default();
        let mut sep2_status = ActuatorStatus::default();
        let mut depl1_status = ActuatorStatus::default();
        let mut depl2_status = ActuatorStatus::default();
        let mut steering = SteeringStatuses::default();
        let mut steering_watchdog_status = WatchdogState::default();
        let mut arming_state = ArmingState::default();
        let mut common = StatusCommonMessage::default();

        loop {
            if last + STATUS_CREATION_INTERVAL <= Instant::now() {
                last = Instant::now();
                //try to update separation status
                if let Some(data) = separation_status_rx.try_changed() {
                    [sep1_status, sep2_status] = data;
                }
                //try to update deployment status
                if let Some(data) = deployment_status_rx.try_changed() {
                    [depl1_status, depl2_status] = data;
                }
                //try to update steering status
                if let Some(data) = steering_status_rx.try_changed() {
                    steering = data;
                }
                //try to update watchdog status
                if let Some(data) = steering_watchdog_status_rx.try_changed() {
                    steering_watchdog_status = data;
                }

                if let Some(data) = arming_state_rx.try_changed() {
                    arming_state = data;
                }

                common.micros_since_restart = Instant::as_micros(&Instant::now());

                //now actually construct the REC board status message with the data collected
                let msg = RecoveryBoardStatus {
                    common: common.clone(),
                    sep1_status,
                    sep2_status,
                    depl1_status,
                    depl2_status,
                    steering_left_status: steering.left,
                    steering_right_status: steering.right,
                    steering_watchdog_status,
                    arming_state,
                };
                info!("status: {}", msg);
                transmit_logged(can_tx, dp_recovery_board::Message::BoardStatus(msg)).await;
            } else {
                Timer::after_millis(25).await;
            }
        }
    };

    let separation_response_task = async {
        let mut separation_triggered_rx = SEPARATION_OCCURRED.receiver().unwrap();
        loop {
            let rx = separation_triggered_rx.changed().await;
            if rx {
                transmit_logged(can_tx, dp_recovery_board::Message::SeparationOccurred).await;
            }
        }
    };

    let deployment_response_task = async {
        let mut deployment_triggered_rx = DEPLOYMENT_OCCURRED.receiver().unwrap();
        loop {
            let rx = deployment_triggered_rx.changed().await;
            if rx {
                transmit_logged(can_tx, dp_recovery_board::Message::DeploymentOccurred).await;
            }
        }
    };

    join3(
        status_creation_task,
        deployment_response_task,
        separation_response_task,
    )
    .await;
}

/// Send a message on CAN, bounded by [`CAN_TX_TIMEOUT`], and log the outcome.
async fn transmit_logged(
    can_tx: &Mutex<CriticalSectionRawMutex, CanTx<'static>>,
    msg: dp_recovery_board::Message,
) {
    let mut tx = can_tx.lock().await;
    match with_timeout(CAN_TX_TIMEOUT, tx.transmit(msg)).await {
        Ok(Ok(_)) => trace!("sent CAN message"),
        Ok(Err(err)) => error!("CAN TX error: {:?}", err),
        Err(_) => error!("CAN TX timed out after {} ms", CAN_TX_TIMEOUT),
    }
}

/// This serves as both an example of how to use [`ReceivedMessage`] as well as a santiy check that it works.
#[test]
fn sanity_check() {
    use data_core::can::hal::{CanDecode, CanEncode};
    let msg =
        dp_recovery_board::Message::SteeringTargetPositions(dp_recovery_board::SteeringPositions {
            left_pos: -132,
            right_pos: 32,
        });
    let mut buf = [0; 64];
    let (id, len) = msg.encode_into(&mut buf).expect("need to be serializable");
    let sparse_msg =
        ReceivedMessage::from_parts(id, &buf[..(len as usize)]).expect("neeed to decode again");

    assert!(matches!(
        sparse_msg,
        ReceivedMessage::SteeringTargetPositions(dp_recovery_board::SteeringPositions {
            left_pos: -132,
            right_pos: 32,
        })
    ))
}
