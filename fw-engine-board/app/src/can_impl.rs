use crate::globals::STATE;
use crate::sensors::CAN_BOARD_STATUS_FREQ_HZ;
use can_utils::collector::Collector;
use can_utils::rxtx::TypedCanReceive as _;
use data_core::can::hal::CanDecode as _;
use datatypes::status::{ArmingState, BoardId, SensorStatus, StatusCommonMessage};
use embassy_futures::yield_now;
use embassy_stm32::can::CanRx;
use embassy_time::{Duration, Instant, Ticker};
use embedded_utils::fmt::*;

const THIS_BOARD_ID: BoardId = BoardId::EngineControlBoard;

data_core::can::sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
        FuelMainValveControlFC(dp_engine_control_board::Message::FuelMainValveControlFC),
        FuelMainValveControlRFS(dp_engine_control_board::Message::FuelMainValveControlRFS),
        OxidizerMainValveControlFC(dp_engine_control_board::Message::OxidizerMainValveControlFC),
        OxidizerMainValveControlRFS(dp_engine_control_board::Message::OxidizerMainValveControlRFS),
    }
}

const __ASSERT_LEN_OK: () = {
    const FILTER_COUNT: usize = 28;
    if ReceivedMessage::SUPPORTED_IDS.len() >= FILTER_COUNT - 1 {
        core::panic!("Too many receiving can ids");
    }
};

#[embassy_executor::task]
pub async fn can_rx_task(mut can_rx: CanRx<'static>) -> ! {
    loop {
        match can_rx.recv().await {
            Ok(ReceivedMessage::ResetAll(_)) => {
                warn!("[CAN Task] Received ResetAll message, resetting EngineControlBoard");
                reset_now();
            }
            Ok(ReceivedMessage::ResetSpecific(board)) => {
                if board == THIS_BOARD_ID {
                    warn!(
                        "[CAN Task] Received ResetSpecific message, resetting EngineControlBoard"
                    );
                    reset_now();
                }
            }
            Ok(msg) => {
                let _ = STATE.update_from(msg);
            }
            Err(err) => {
                error!("CAN RX error: {:?}", err);
            }
        }
        yield_now().await;
    }
}

#[embassy_executor::task]
pub async fn board_status_update_task() -> ! {
    let start = Instant::now();
    let mut status_ticker = Ticker::every(Duration::from_millis(
        1000 / CAN_BOARD_STATUS_FREQ_HZ as u64,
    ));
    let build_info = crate::build_info::BUILD_INFO.get();
    STATE.build_info.sender().send(build_info.clone());
    let mut thermocouple_status_watch = STATE.thermocouple_status.receiver().unwrap();
    let mut arming_state_watch = STATE.arming_state.receiver().unwrap();
    let mut thermocouple_status = SensorStatus::Online;
    let mut armed = ArmingState::Safe;

    loop {
        if let Some(status) = thermocouple_status_watch.try_changed() {
            thermocouple_status = status;
        }
        if let Some(state) = arming_state_watch.try_changed() {
            armed = state;
        }

        STATE
            .board_status
            .sender()
            .send(dp_engine_control_board::EngineControlBoardStatus {
                common: StatusCommonMessage {
                    errors: 0,
                    micros_since_restart: start.elapsed().as_micros(),
                },
                thermocouple_status,
                armed,
            });
        status_ticker.next().await;
    }
}

fn reset_now() {
    cortex_m::peripheral::SCB::sys_reset();
}
