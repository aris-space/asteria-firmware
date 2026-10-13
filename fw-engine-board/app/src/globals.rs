use crate::buzzer::BuzzerState;
use crate::drivers::WATCH;
use crate::drivers::solenoid_detection::SolenoidStates;
use crate::sensors::solenoid_current::SolenoidCurrentMeasurements;
use can_utils::broadcast::Broadcast;
use can_utils::collector::Collector;
use datatypes::actuator::NormallyClosedValve;
use datatypes::status::{ArmingState, BuildInformationCommon, SensorStatus};
use datatypes::units::{BarG, Celsius};
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
        map = "dp_engine_control_board::Message::ChamberPressure(#value)",
        min_freq_hz = 15.0,
        max_freq_hz = 22.0
    )]
    pub eng_cc_p: Watch<ThreadModeRawMutex, BarG, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::InjectorFuelPressure(#value)",
        min_freq_hz = 15.0,
        max_freq_hz = 22.0
    )]
    pub fss_inj_p: Watch<ThreadModeRawMutex, BarG, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::InjectorOxidizerPressure(#value)",
        min_freq_hz = 15.0,
        max_freq_hz = 22.0
    )]
    pub oss_inj_p: Watch<ThreadModeRawMutex, BarG, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::InjectorFuelTemperature(#value)",
        min_freq_hz = 8.0,
        max_freq_hz = 12.0
    )]
    pub fss_inj_t: Watch<ThreadModeRawMutex, Celsius, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::OxidizerTankTemperature(#value)",
        min_freq_hz = 8.0,
        max_freq_hz = 12.0
    )]
    pub oss_tnk_t: Watch<ThreadModeRawMutex, Celsius, 5>,
    // Valves
    #[broadcast(
        map = "dp_engine_control_board::Message::FuelMainValveState(#value)",
        min_freq_hz = 4.0,
        max_freq_hz = 6.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::FuelMainValveControlFC(#value) | crate::can_impl::ReceivedMessage::FuelMainValveControlRFS(#value)"
    )]
    pub fuel_main_control: Watch<ThreadModeRawMutex, NormallyClosedValve, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::OxidizerMainValveState(#value)",
        min_freq_hz = 4.0,
        max_freq_hz = 6.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::OxidizerMainValveControlFC(#value) | crate::can_impl::ReceivedMessage::OxidizerMainValveControlRFS(#value)"
    )]
    pub oxidizer_main_control: Watch<ThreadModeRawMutex, NormallyClosedValve, 5>,
    // Status
    #[broadcast(
        map = "dp_engine_control_board::Message::BoardStatus(#value)",
        min_freq_hz = 0.8,
        max_freq_hz = 1.2
    )]
    pub board_status:
        Watch<ThreadModeRawMutex, dp_engine_control_board::EngineControlBoardStatus, 5>,
    #[broadcast(
        map = "dp_engine_control_board::Message::BuildInfo(#value)",
        min_freq_hz = 0.15,
        max_freq_hz = 0.25
    )]
    pub build_info: Watch<ThreadModeRawMutex, BuildInformationCommon, 5>,
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
            eng_cc_p: Watch::new(),
            fss_inj_p: Watch::new(),
            oss_inj_p: Watch::new(),
            fss_inj_t: Watch::new(),
            oss_tnk_t: Watch::new(),
            fuel_main_control: Watch::new(),
            oxidizer_main_control: Watch::new(),
            board_status: Watch::new(),
            build_info: Watch::new(),
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
