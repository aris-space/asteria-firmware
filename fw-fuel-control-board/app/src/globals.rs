use crate::buzzer::BuzzerState;
use crate::drivers::WATCH;
use crate::drivers::analog_pressure::FuelTankPressureMeasurement;
use crate::drivers::solenoid_detection::SolenoidStates;
use crate::sensors::solenoid_current::SolenoidCurrentMeasurements;
use can_utils::broadcast::Broadcast;
use can_utils::collector::Collector;
use datatypes::actuator::{DPRValve, NormallyOpenValve};
use datatypes::status::BuildInformationCommon;
use datatypes::units::BarG;
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
        map = "dp_fuel_control_board::Message::PressurizationLinePressure(#value)",
        min_freq_hz = 20.0,
        max_freq_hz = 20.0
    )]
    pub pressurization_pressure: Watch<ThreadModeRawMutex, BarG, 5>,
    #[broadcast(
        map = "dp_fuel_control_board::Message::FuelTankPressure(dp_fuel_control_board::FuelTankPressure { fuel_tank_pressure_sensor_1: #value.fuel_tank_pressure_1, fuel_tank_pressure_sensor_2: #value.fuel_tank_pressure_2, fuel_tank_pressure_filtered: #value.dpr_pressure })",
        min_freq_hz = 20.0,
        max_freq_hz = 20.0
    )]
    pub fuel_tank_pressure: Watch<ThreadModeRawMutex, FuelTankPressureMeasurement, 5>,
    // DPR
    #[broadcast(
        map = "dp_fuel_control_board::Message::FuelDprValveState(#value)",
        min_freq_hz = 5.0,
        max_freq_hz = 5.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::FuelDprValveControlFC(#value) | crate::can_impl::ReceivedMessage::FuelDprValveControlRFS(#value)"
    )]
    pub dpr_control_loop: Watch<ThreadModeRawMutex, DPRValve, 5>,
    // Valves
    #[broadcast(
        map = "dp_fuel_control_board::Message::PressurizationVentValveState(#value)",
        min_freq_hz = 5.0,
        max_freq_hz = 5.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::PressurizationVentValveControlFC(#value) | crate::can_impl::ReceivedMessage::PressurizationVentValveControlRFS(#value)"
    )]
    pub pressurization_vent_control: Watch<ThreadModeRawMutex, NormallyOpenValve, 5>,
    #[broadcast(
        map = "dp_fuel_control_board::Message::FuelVentValveState(#value)",
        min_freq_hz = 5.0,
        max_freq_hz = 5.0
    )]
    #[collector(
        pattern = "crate::can_impl::ReceivedMessage::FuelVentValveControlFC(#value) | crate::can_impl::ReceivedMessage::FuelVentValveControlRFS(#value)"
    )]
    pub fuel_vent_control: Watch<ThreadModeRawMutex, NormallyOpenValve, 5>,
    #[broadcast(
        map = "dp_fuel_control_board::Message::BoardStatus(#value)",
        min_freq_hz = 1.0,
        max_freq_hz = 1.0
    )]
    pub board_status: Watch<ThreadModeRawMutex, dp_fuel_control_board::FuelControlBoardStatus, 5>,
    #[broadcast(
        map = "dp_fuel_control_board::Message::BuildInfo(#value)",
        min_freq_hz = 0.2,
        max_freq_hz = 0.2
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
}

impl BoardState {
    const fn new() -> Self {
        Self {
            pressurization_pressure: Watch::new(),
            fuel_tank_pressure: Watch::new(),
            dpr_control_loop: Watch::new(),
            pressurization_vent_control: Watch::new(),
            fuel_vent_control: Watch::new(),
            board_status: Watch::new(),
            build_info: Watch::new(),
            pressure_bus_status: Watch::new(),
            buzzer: Watch::new(),
            solenoid_states: Watch::new(),
            solenoid_currents: Watch::new(),
        }
    }
}

pub static STATE: BoardState = BoardState::new();
