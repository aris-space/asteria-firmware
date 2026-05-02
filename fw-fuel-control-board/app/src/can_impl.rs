use crate::globals::STATE;
use crate::sensors::CAN_BOARD_STATUS_FREQ_HZ;
use can_utils::collector::Collector;
use can_utils::rxtx::TypedCanReceive as _;
use can_utils::setup::setup_can as setup_can_with_filters;
use data_core::can::hal::CanDecode as _;
use datatypes::status::{BoardId, SensorStatus, StatusCommonMessage};
use embassy_futures::yield_now;
use embassy_stm32::can::{Can, CanRx, RxPin, TxPin};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::{Peri, can};
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
    }
}

const __ASSERT_LEN_OK: () = {
    const FILTER_COUNT: usize = 28;
    if ReceivedMessage::SUPPORTED_IDS.len() >= FILTER_COUNT - 1 {
        core::panic!("Too many receiving can ids");
    }
};

pub fn setup_can<'a, T: can::Instance>(
    peri: Peri<'a, T>,
    rx: Peri<'a, impl RxPin<T>>,
    tx: Peri<'a, impl TxPin<T>>,
    irqs: impl Binding<T::IT0Interrupt, can::IT0InterruptHandler<T>>
    + Binding<T::IT1Interrupt, can::IT1InterruptHandler<T>>
    + 'a,
) -> Can<'a> {
    setup_can_with_filters(peri, rx, tx, irqs, ReceivedMessage::SUPPORTED_IDS)
}

#[embassy_executor::task]
pub async fn can_rx_task(mut can_rx: CanRx<'static>) -> ! {
    loop {
        match recv_message(&mut can_rx).await {
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
pub async fn can_tx_task() -> ! {
    let start = Instant::now();
    let mut status_ticker = Ticker::every(Duration::from_millis(
        1000 / CAN_BOARD_STATUS_FREQ_HZ as u64,
    ));
    let build_info = crate::build_info::BUILD_INFO.get();
    STATE.build_info.sender().send(build_info.clone());
    let mut pressure_bus_status = STATE.pressure_bus_status.receiver().unwrap();
    let mut pressure_status = SensorStatus::Online;

    loop {
        if let Some(status) = pressure_bus_status.try_changed() {
            pressure_status = status;
        }

        STATE
            .board_status
            .sender()
            .send(dp_fuel_control_board::FuelControlBoardStatus {
                common: StatusCommonMessage {
                    errors: 0,
                    micros_since_restart: start.elapsed().as_micros(),
                },
                thermocouple_status: SensorStatus::Online,
                pressure_bus: pressure_status,
            });

        status_ticker.next().await;
    }
}

fn reset_now() {
    cortex_m::peripheral::SCB::sys_reset();
}

async fn recv_message(can_rx: &mut CanRx<'_>) -> Result<ReceivedMessage, ()> {
    can_rx.recv().await.map_err(|_| ())
}
