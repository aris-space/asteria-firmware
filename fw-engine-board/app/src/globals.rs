use crate::buzzer::BuzzerState;
use crate::drivers::WATCH;
use crate::drivers::solenoid_detection::SolenoidStates;
use crate::sensors::solenoid_current::SolenoidCurrentMeasurements;
use can_utils::broadcast::Broadcast;
use can_utils::collector::Collector;
use datatypes::actuator::NormallyClosedValve;
use datatypes::status::{ArmingState, BuildInformationCommon, SensorStatus};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;

#[derive(Broadcast, Collector)]
#[broadcast(loop_type = "can_utils::broadcast::ResponsiveLoop")]
#[collector(
    message_type = "crate::can_impl::ReceivedMessage",
    update_expr = "#field.sender().send(#value);"
)]
pub struct BoardState {
    // Sensors
    #[broadcast(
        map = "dp_engine_control_board::Message::EnginePressure(#value)",
        min_freq_hz = 20.0,
        max_freq_hz = 20.0
    )]
    pub engine_pressure: Watch<ThreadModeRawMutex, dp_engine_control_board::EnginePressure, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::EngineBayTemperature(#value)",
        min_freq_hz = 10.0,
        max_freq_hz = 10.0
    )]
    pub engine_bay_temperature:
        Watch<ThreadModeRawMutex, dp_engine_control_board::EngineBayTemperature, 5>,
    // Valves
    #[broadcast(
        map = "dp_engine_control_board::Message::FuelMainValveState(#value)",
        min_freq_hz = 5.0,
        max_freq_hz = 5.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::FuelMainValveControlFC(#value) | crate::can_impl::ReceivedMessage::FuelMainValveControlRFS(#value)"
    )]
    pub fuel_main_control: Watch<ThreadModeRawMutex, NormallyClosedValve, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::OxidizerMainValveState(#value)",
        min_freq_hz = 5.0,
        max_freq_hz = 5.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::OxidizerMainValveControlFC(#value) | crate::can_impl::ReceivedMessage::OxidizerMainValveControlRFS(#value)"
    )]
    pub oxidizer_main_control: Watch<ThreadModeRawMutex, NormallyClosedValve, 5>,
    // Status
    #[broadcast(
        map = "dp_engine_control_board::Message::BoardStatus(#value)",
        min_freq_hz = 1.0,
        max_freq_hz = 1.0
    )]
    pub board_status:
        Watch<ThreadModeRawMutex, dp_engine_control_board::EngineControlBoardStatus, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::BuildInfo(#value)",
        min_freq_hz = 0.2,
        max_freq_hz = 0.2
    )]
    pub build_info: Watch<ThreadModeRawMutex, BuildInformationCommon, 5>,
    // Events
    #[broadcast(
        map = "dp_engine_control_board::Message::FiringAborted",
        min_freq_hz = 0.0,
        max_freq_hz = 10.0
    )]
    pub firing_aborted: Watch<ThreadModeRawMutex, bool, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::FiringCompleted",
        min_freq_hz = 0.0,
        max_freq_hz = 10.0
    )]
    pub firing_completed: Watch<ThreadModeRawMutex, bool, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::FiringInitiated",
        min_freq_hz = 0.0,
        max_freq_hz = 10.0
    )]
    pub firing_initiated: Watch<ThreadModeRawMutex, bool, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::IgnitionDetected",
        min_freq_hz = 0.0,
        max_freq_hz = 10.0
    )]
    pub ignition_detected: Watch<ThreadModeRawMutex, bool, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::CombustionDetected",
        min_freq_hz = 0.0,
        max_freq_hz = 10.0
    )]
    pub combustion_detected: Watch<ThreadModeRawMutex, bool, 5>,
    // Board-local state
    pub thermocouple_status: Watch<ThreadModeRawMutex, SensorStatus, WATCH>,
    pub arming_state: Watch<ThreadModeRawMutex, ArmingState, WATCH>,
    pub buzzer: Watch<ThreadModeRawMutex, BuzzerState, WATCH>,
    pub engine_chamber_pressure: Watch<ThreadModeRawMutex, f32, WATCH>,
    pub solenoid_states: Watch<ThreadModeRawMutex, SolenoidStates, WATCH>,
    // TODO: Broadcast this once the corresponding data-definition messages exist.
    pub solenoid_currents: Watch<ThreadModeRawMutex, SolenoidCurrentMeasurements, WATCH>,
}

impl BoardState {
    const fn new() -> Self {
        Self {
            engine_pressure: Watch::new(),
            engine_bay_temperature: Watch::new(),
            fuel_main_control: Watch::new(),
            oxidizer_main_control: Watch::new(),
            board_status: Watch::new(),
            build_info: Watch::new(),
            firing_aborted: Watch::new(),
            firing_completed: Watch::new(),
            firing_initiated: Watch::new(),
            ignition_detected: Watch::new(),
            combustion_detected: Watch::new(),
            thermocouple_status: Watch::new(),
            arming_state: Watch::new(),
            buzzer: Watch::new(),
            engine_chamber_pressure: Watch::new(),
            solenoid_states: Watch::new(),
            solenoid_currents: Watch::new(),
        }
    }
}

pub static STATE: BoardState = BoardState::new();
