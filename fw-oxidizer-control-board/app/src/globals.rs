use crate::buzzer::BuzzerState;
use crate::drivers::WATCH;
use crate::drivers::solenoid_detection::SolenoidStates;
use can_utils::broadcast::Broadcast;
use can_utils::collector::Collector;
use datatypes::actuator::{DPRValve, NormallyOpenValve};
use datatypes::status::BuildInformationCommon;
use datatypes::units::BarG;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;

use crate::sensors::solenoid_current::SolenoidCurrentMeasurements;

#[derive(Broadcast, Collector)]
#[broadcast(loop_type = "can_utils::broadcast::ResponsiveLoop")]
#[collector(
    message_type = "crate::can_impl::ReceivedMessage",
    update_expr = "#field.sender().send(#value);"
)]
pub struct BoardState {
    // Pressure sensors
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::OxidizerTankPressureSensor1(#value)",
        min_freq_hz = 15.0,
        max_freq_hz = 22.0
    )]
    pub oxidizer_tank_pressure_sensor_1: Watch<ThreadModeRawMutex, BarG, 5>,
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::OxidizerTankPressureSensor2(#value)",
        min_freq_hz = 15.0,
        max_freq_hz = 22.0
    )]
    pub oxidizer_tank_pressure_sensor_2: Watch<ThreadModeRawMutex, BarG, 5>,
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::OxidizerTankDifferentialPressure(#value)",
        min_freq_hz = 15.0,
        max_freq_hz = 22.0
    )]
    pub oxidizer_tank_differential_pressure: Watch<ThreadModeRawMutex, BarG, 5>,
    // DPR
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::OxidizerDprValveState(#value)",
        min_freq_hz = 4.0,
        max_freq_hz = 6.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::OxidizerDprValveControlFC(#value) | crate::can_impl::ReceivedMessage::OxidizerDprValveControlRFS(#value)"
    )]
    pub dpr_control_loop: Watch<ThreadModeRawMutex, DPRValve, 5>,
    // Valves
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::OxidizerVentValveState(#value)",
        min_freq_hz = 4.0,
        max_freq_hz = 6.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::OxidizerVentValveControlFC(#value) | crate::can_impl::ReceivedMessage::OxidizerVentValveControlRFS(#value)"
    )]
    pub oxidizer_vent_control: Watch<ThreadModeRawMutex, NormallyOpenValve, 5>,
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::BoardStatus(#value)",
        min_freq_hz = 0.8,
        max_freq_hz = 1.2
    )]
    pub board_status:
        Watch<ThreadModeRawMutex, dp_oxidizer_control_board::OxidizerControlBoardStatus, 5>,
    #[broadcast(
        map = "dp_oxidizer_control_board::Message::BuildInfo(#value)",
        min_freq_hz = 0.15,
        max_freq_hz = 0.25
    )]
    pub build_info: Watch<ThreadModeRawMutex, BuildInformationCommon, 5>,
    // Board state
    pub pressure_bus_status: Watch<ThreadModeRawMutex, datatypes::status::SensorStatus, WATCH>,
    // Buzzer
    pub buzzer: Watch<ThreadModeRawMutex, BuzzerState, WATCH>,
    // Solenoid detection
    pub solenoid_states: Watch<ThreadModeRawMutex, SolenoidStates, WATCH>,
    // Solenoid currents
    // TODO: Broadcast this once the corresponding data-definition messages exist.
    pub solenoid_currents: Watch<ThreadModeRawMutex, SolenoidCurrentMeasurements, WATCH>,
    /// Raw, unfiltered tank pressure used as the DPR control loop input.
    pub dpr_pressure: Watch<ThreadModeRawMutex, f32, WATCH>,
}

impl BoardState {
    const fn new() -> Self {
        Self {
            oxidizer_tank_pressure_sensor_1: Watch::new(),
            oxidizer_tank_pressure_sensor_2: Watch::new(),
            oxidizer_tank_differential_pressure: Watch::new(),
            dpr_control_loop: Watch::new(),
            oxidizer_vent_control: Watch::new(),
            board_status: Watch::new(),
            build_info: Watch::new(),
            pressure_bus_status: Watch::new(),
            buzzer: Watch::new(),
            solenoid_states: Watch::new(),
            solenoid_currents: Watch::new(),
            dpr_pressure: Watch::new(),
        }
    }
}

pub static STATE: BoardState = BoardState::new();
