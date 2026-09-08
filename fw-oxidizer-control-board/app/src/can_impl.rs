use crate::buzzer::BuzzerState;
use crate::globals::STATE;
use crate::sensors::CAN_BOARD_STATUS_FREQ_HZ;
use can_utils::collector::Collector;
use can_utils::rxtx::TypedCanReceive as _;
use data_core::can::hal::CanDecode as _;
use datatypes::status::{BoardId, DprGainInfo, DprLoopInfo, SensorStatus, StatusCommonMessage};
use dpr::dpr::{GAINS, MAX_TIME_MS, MIN_TIME_MS};
use embassy_futures::yield_now;
use embassy_stm32::can::CanRx;
use embassy_time::{Duration, Instant, Ticker};
use embedded_utils::fmt::*;

const THIS_BOARD_ID: BoardId = BoardId::OxidizerControlBoard;

data_core::can::sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
        OxidizerDprValveControlFC(dp_oxidizer_control_board::Message::OxidizerDprValveControlFC),
        OxidizerDprValveControlRFS(dp_oxidizer_control_board::Message::OxidizerDprValveControlRFS),
        OxidizerVentValveControlFC(dp_oxidizer_control_board::Message::OxidizerVentValveControlFC),
        OxidizerVentValveControlRFS(dp_oxidizer_control_board::Message::OxidizerVentValveControlRFS),
        OxidizerDprGainP(dp_oxidizer_control_board::Message::OxidizerDprGainP),
        OxidizerDprGainI(dp_oxidizer_control_board::Message::OxidizerDprGainI),
        OxidizerDprGainD(dp_oxidizer_control_board::Message::OxidizerDprGainD),
        OxidizerDprMinOpeningTime(dp_oxidizer_control_board::Message::OxidizerDprMinOpeningTime),
        OxidizerDprMaxOpeningTime(dp_oxidizer_control_board::Message::OxidizerDprMaxOpeningTime),
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
    let dpr_gain_sender = STATE.dpr_gain.sender();

    let mut dpr_gain = DprGainInfo {
        p: GAINS.p,
        i: GAINS.i,
        d: GAINS.d,
        min_ms: MIN_TIME_MS,
        max_ms: MAX_TIME_MS,
    };

    loop {
        match can_rx.recv().await {
            Ok(ReceivedMessage::ResetAll(_)) => {
                warn!("[CAN Task] Received ResetAll message, resetting OxidizerControlBoard");
                reset_now();
            }
            Ok(ReceivedMessage::ResetSpecific(board)) => {
                if board == THIS_BOARD_ID {
                    warn!(
                        "[CAN Task] Received ResetSpecific message, resetting OxidizerControlBoard"
                    );
                    reset_now();
                }
            }
            Ok(msg) => match msg {
                ReceivedMessage::OxidizerDprGainP(p) => {
                    dpr_gain.p = p;
                    dpr_gain_sender.send(dpr_gain);
                }
                ReceivedMessage::OxidizerDprGainI(i) => {
                    dpr_gain.i = i;
                    dpr_gain_sender.send(dpr_gain);
                }
                ReceivedMessage::OxidizerDprGainD(d) => {
                    dpr_gain.d = d;
                    dpr_gain_sender.send(dpr_gain);
                }
                ReceivedMessage::OxidizerDprMinOpeningTime(min_ms) => {
                    dpr_gain.min_ms = min_ms;
                    dpr_gain_sender.send(dpr_gain);
                }
                ReceivedMessage::OxidizerDprMaxOpeningTime(max_ms) => {
                    dpr_gain.max_ms = max_ms;
                    dpr_gain_sender.send(dpr_gain);
                }
                _ => {
                    let _ = STATE.update_from(msg);
                }
            },
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
    let mut pressure_bus_status = STATE.pressure_bus_status.receiver().unwrap();
    let mut dpr_info_receiver = STATE.dpr_info.receiver().unwrap();
    let buzzer_sender = STATE.buzzer.sender();
    let mut dpr_gain_receiver = STATE.dpr_gain.receiver().unwrap();
    let mut pressure_status = SensorStatus::Online;
    let mut dpr_loop_info = DprLoopInfo::default();
    let mut dpr_gain = DprGainInfo {
        p: GAINS.p,
        i: GAINS.i,
        d: GAINS.d,
        min_ms: MIN_TIME_MS,
        max_ms: MAX_TIME_MS,
    };

    loop {
        if let Some(status) = pressure_bus_status.try_changed() {
            pressure_status = status;
        }
        if let Some(status) = dpr_info_receiver.try_changed() {
            dpr_loop_info = status;
            buzzer_sender.send(match status {
                DprLoopInfo::ActiveOverPressure => BuzzerState::Error,
                _ => BuzzerState::Idle,
            });
        }
        if let Some(gain) = dpr_gain_receiver.try_changed() {
            dpr_gain = gain;
        }

        STATE
            .board_status
            .sender()
            .send(dp_oxidizer_control_board::OxidizerControlBoardStatus {
                common: StatusCommonMessage {
                    errors: 0,
                    micros_since_restart: start.elapsed().as_micros(),
                },
                thermocouple_status: SensorStatus::Online,
                pressure_bus: pressure_status,
                dpr_loop_info,
                dpr_gain_info: dpr_gain,
            });

        status_ticker.next().await;
    }
}

fn reset_now() {
    cortex_m::peripheral::SCB::sys_reset();
}
