use can_utils::broadcast::Broadcast;
use core::sync::atomic::Ordering;
use data_core::can::{hal::CanDecode, sparse_decodable_can_message};
use datatypes::status::{BoardId, BuildInformationCommon, StatusCommonMessage};
use datatypes::units::RailStatus;
use dp_backplane::{ActivePowerSource, Message};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker};
use embedded_utils::ERROR_COUNT;

sparse_decodable_can_message! {
  enum ReceivedMessage {
    ResetAll(dp_system_management::Message::ResetAll),
    ResetSpecific(dp_system_management::Message::ResetSpecific),
    UTCTimeUpdate(dp_system_management::Message::UTCTimeUpdate),
  }
}

const __ASSERT_LEN_OK: () = {
    const FILTER_COUNT: usize = 28;
    if ReceivedMessage::SUPPORTED_IDS.len() >= FILTER_COUNT - 1 {
        core::panic!("Too many receiving can ids");
    }
};

pub const THIS_BOARD_ID: BoardId = BoardId::Backplane;

#[derive(Broadcast)]
#[broadcast(loop_type = "can_utils::broadcast::ResponsiveLoop")]
pub struct Outputs {
    #[broadcast(
        map = "Message::RailStatus5V(#value)",
        min_freq_hz = 5.0,
        max_freq_hz = 10.5
    )]
    pub rail_5v: Watch<CriticalSectionRawMutex, RailStatus, 2>,
    #[broadcast(
        map = "Message::RailStatus24V(#value)",
        min_freq_hz = 5.0,
        max_freq_hz = 10.5
    )]
    pub rail_24v: Watch<CriticalSectionRawMutex, RailStatus, 2>,
    #[broadcast(
        map = "Message::ActivePowerSource(#value)",
        min_freq_hz = 0.8,
        max_freq_hz = 1.0
    )]
    pub active_power_source: Watch<CriticalSectionRawMutex, ActivePowerSource, 1>,
    #[broadcast(
        map = "Message::BuildInfo(#value)",
        min_freq_hz = 0.2,
        max_freq_hz = 0.2
    )]
    pub build_info: Watch<CriticalSectionRawMutex, BuildInformationCommon, 1>,
    #[broadcast(
        map = "Message::BoardStatus(#value)",
        min_freq_hz = 0.0,
        max_freq_hz = 1.0
    )]
    pub status: Watch<CriticalSectionRawMutex, StatusCommonMessage, 1>,
}

pub static OUTPUTS: Outputs = Outputs {
    rail_5v: Watch::new(),
    rail_24v: Watch::new(),
    active_power_source: Watch::new(),
    build_info: Watch::new(),
    status: Watch::new(),
};

#[embassy_executor::task]
pub async fn can_board_status_task() {
    let status = OUTPUTS.status.sender();
    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        let current = StatusCommonMessage {
            errors: ERROR_COUNT.load(Ordering::Relaxed),
            micros_since_restart: Instant::now().as_micros(),
        };

        status.send(current);
        ticker.next().await;
    }
}
