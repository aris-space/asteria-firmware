use crate::buzzer::BuzzerState;
use crate::drivers::WATCH;
use can_utils::broadcast::Broadcast;
use can_utils::collector::Collector;
use datatypes::actuator::{DPRValve, NormallyClosedValve};
use datatypes::status::BuildInformationCommon;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;

#[derive(Broadcast, Collector)]
#[broadcast(loop_type = "can_utils::broadcast::ResponsiveLoop")]
#[collector(
    message_type = "crate::can_impl::ReceivedMessage",
    update_expr = "#field.sender().send(#value);"
)]
pub struct BoardState {
    // Pressure sensors
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::OxidizerTankPressure(#value)",
        min_freq_hz = 20.0,
        max_freq_hz = 20.0
    )]
    pub oxidizer_tank_pressure:
        Watch<ThreadModeRawMutex, dp_oxidizer_control_board::OxidizerTankPressure, 5>,
    // DPR
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::OxidizerDprValveState(#value)",
        min_freq_hz = 5.0,
        max_freq_hz = 5.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::OxidizerDprValveControlFC(#value) | crate::can_impl::ReceivedMessage::OxidizerDprValveControlRFS(#value)"
    )]
    pub dpr_control_loop: Watch<ThreadModeRawMutex, DPRValve, 5>,
    // Valves
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::OxidizerVentValveState(#value)",
        min_freq_hz = 5.0,
        max_freq_hz = 5.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::OxidizerVentValveControlFC(#value) | crate::can_impl::ReceivedMessage::OxidizerVentValveControlRFS(#value)"
    )]
    pub oxidizer_vent_control: Watch<ThreadModeRawMutex, NormallyClosedValve, 5>,
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::BoardStatus(#value)",
        min_freq_hz = 1.0,
        max_freq_hz = 1.0
    )]
    pub board_status:
        Watch<ThreadModeRawMutex, dp_oxidizer_control_board::OxidizerControlBoardStatus, 5>,
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::BuildInfo(#value)",
        min_freq_hz = 0.2,
        max_freq_hz = 0.2
    )]
    pub build_info: Watch<ThreadModeRawMutex, BuildInformationCommon, 5>,
    // Board state
    pub pressure_bus_status: Watch<ThreadModeRawMutex, datatypes::status::SensorStatus, WATCH>,
    pub thermocouple_status: Watch<ThreadModeRawMutex, datatypes::status::SensorStatus, WATCH>,
    // Buzzer
    pub buzzer: Watch<ThreadModeRawMutex, BuzzerState, WATCH>,
}

impl BoardState {
    const fn new() -> Self {
        Self {
            oxidizer_tank_pressure: Watch::new(),
            dpr_control_loop: Watch::new(),
            oxidizer_vent_control: Watch::new(),
            board_status: Watch::new(),
            build_info: Watch::new(),
            pressure_bus_status: Watch::new(),
            thermocouple_status: Watch::new(),
            buzzer: Watch::new(),
        }
    }
}

pub static STATE: BoardState = BoardState::new();
