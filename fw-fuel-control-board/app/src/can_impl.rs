use crate::globals::STATE;
use crate::sensors::CAN_BOARD_STATUS_FREQ_HZ;
use can_utils::collector::Collector;
use can_utils::rxtx::TypedCanReceive as _;
use data_core::can::hal::CanDecode as _;
use datatypes::actuator::DPRValve;
use datatypes::status::{BoardId, DprGainInfo, DprLoopInfo, SensorStatus, StatusCommonMessage};
use embassy_futures::yield_now;
use embassy_stm32::can::CanRx;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker};
use embedded_utils::fmt::*;

const THIS_BOARD_ID: BoardId = BoardId::FuelControlBoard;

data_core::can::sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
        FuelDprValveControlFC(dp_fuel_control_board::Message::FuelDprValveControlFC),
        FuelDprValveControlRFS(dp_fuel_control_board::Message::FuelDprValveControlRFS),
        PressurizationVentValveControlFC(dp_fuel_control_board::Message::PressurizationVentValveControlFC),
        PressurizationVentValveControlRFS(dp_fuel_control_board::Message::PressurizationVentValveControlRFS),
        FuelVentValveControlFC(dp_fuel_control_board::Message::FuelVentValveControlFC),
        FuelVentValveControlRFS(dp_fuel_control_board::Message::FuelVentValveControlRFS),
        FuelDprGainP(dp_fuel_control_board::Message::FuelDprGainP),
        FuelDprGainI(dp_fuel_control_board::Message::FuelDprGainI),
        FuelDprGainD(dp_fuel_control_board::Message::FuelDprGainD),
        FuelDprMinOpeningTime(dp_fuel_control_board::Message::FuelDprMinOpeningTime),
        FuelDprMaxOpeningTime(dp_fuel_control_board::Message::FuelDprMaxOpeningTime),
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
    let mut dpr_gain_sender = STATE.dpr_gain.sender();

    use dpr::dpr::{GAINS, MAX_TIME_MS, MIN_TIME_MS};
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
                warn!("[CAN Task] Received ResetAll message, resetting FuelControlBoard");
                reset_now();
            }
            Ok(ReceivedMessage::ResetSpecific(board)) => {
                if board == THIS_BOARD_ID {
                    warn!("[CAN Task] Received ResetSpecific message, resetting FuelControlBoard");
                    reset_now();
                }
            }
            Ok(msg) => match msg {
                ReceivedMessage::FuelDprGainP(p) => {
                    dpr_gain.p = p;
                    let _ = dpr_gain_sender.send(dpr_gain);
                }
                ReceivedMessage::FuelDprGainI(i) => {
                    dpr_gain.i = i;
                    let _ = dpr_gain_sender.send(dpr_gain);
                }
                ReceivedMessage::FuelDprGainD(d) => {
                    dpr_gain.d = d;
                    let _ = dpr_gain_sender.send(dpr_gain);
                }
                ReceivedMessage::FuelDprMinOpeningTime(min_ms) => {
                    dpr_gain.min_ms = min_ms;
                    let _ = dpr_gain_sender.send(dpr_gain);
                }
                ReceivedMessage::FuelDprMaxOpeningTime(max_ms) => {
                    dpr_gain.max_ms = max_ms;
                    let _ = dpr_gain_sender.send(dpr_gain);
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
    let mut dpr_info_receiver = STATE.dpr_info.receiver().unwrap();
    let mut dpr_status = DprLoopInfo::default();

    let mut dpr_gain_receiver = STATE.dpr_gain.receiver().unwrap();
    let mut dpr_gain = DprGainInfo::default();

    let build_info = crate::build_info::BUILD_INFO.get();
    STATE.build_info.sender().send(build_info.clone());
    let mut pressure_bus_status = STATE.pressure_bus_status.receiver().unwrap();
    let mut pressure_status = SensorStatus::Online;

    loop {
        if let Some(status) = pressure_bus_status.try_changed() {
            pressure_status = status;
        }
        if let Some(status) = dpr_info_receiver.try_changed() {
            dpr_status = status;
        }
        if let Some(gain) = dpr_gain_receiver.try_changed() {
            dpr_gain = gain;
        }

        let _ = STATE
            .board_status
            .sender()
            .send(dp_fuel_control_board::FuelControlBoardStatus {
                common: StatusCommonMessage {
                    errors: 0,
                    micros_since_restart: start.elapsed().as_micros(),
                },
                thermocouple_status: SensorStatus::Online,
                pressure_bus: pressure_status,
                dpr_loop_info: dpr_status,
                dpr_gain_info: dpr_gain,
            });

        status_ticker.next().await;
    }
}

fn reset_now() {
    cortex_m::peripheral::SCB::sys_reset();
}
