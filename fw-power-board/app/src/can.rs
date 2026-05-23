use can_utils::broadcast::Broadcast;
use can_utils::rxtx::TypedCanTransmit;
use core::sync::atomic::Ordering;
use datatypes::status::{BoardId, BuildInformationCommon, StatusCommonMessage};
use datatypes::units::RailStatus;
use dp_backplane::Message;
use embassy_stm32::can::CanTx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use embedded_utils::ERROR_COUNT;
use embedded_utils::fmt::{error, trace};

const CAN_SEND_TIMEOUT: Duration = Duration::from_millis(100);
pub const THIS_BOARD_ID: BoardId = BoardId::Backplane;

#[derive(Broadcast)]
#[broadcast(loop_type = "can_utils::broadcast::ResponsiveLoop")]
pub struct Outputs {
    #[broadcast(
        map = "Message::RailStatus5V(#value)",
        min_freq_hz = 8.0,
        max_freq_hz = 10.0
    )]
    pub rail_5v: Watch<CriticalSectionRawMutex, RailStatus, 2>,
    #[broadcast(
        map = "Message::RailStatus24V(#value)",
        min_freq_hz = 8.0,
        max_freq_hz = 10.0
    )]
    pub rail_24v: Watch<CriticalSectionRawMutex, RailStatus, 2>,
    #[broadcast(
        map = "Message::BuildInfo(#value)",
        min_freq_hz = 0.2,
        max_freq_hz = 0.2
    )]
    pub build_info: Watch<CriticalSectionRawMutex, BuildInformationCommon, 1>,
}

pub static OUTPUTS: Outputs = Outputs {
    rail_5v: Watch::new(),
    rail_24v: Watch::new(),
    build_info: Watch::new(),
};

#[embassy_executor::task]
pub async fn can_board_status_task(
    can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>,
) {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        let msg = Message::BoardStatus(StatusCommonMessage {
            errors: ERROR_COUNT.load(Ordering::Relaxed),
            micros_since_restart: Instant::now().as_micros(),
        });
        let mut tx = can_tx.lock().await;
        match with_timeout(CAN_SEND_TIMEOUT, tx.transmit(msg)).await {
            Ok(Ok(())) => {
                trace!("sent backplane status");
            }
            Ok(Err(err)) => {
                error!("CAN TX error: {:?}", err);
            }
            Err(_) => {
                error!("CAN TX timed out after {} ms", CAN_SEND_TIMEOUT.as_millis());
            }
        }
        drop(tx);
        ticker.next().await;
    }
}
