use crate::buzzer::BuzzerState;
use crate::drivers::WATCH;
use crate::drivers::analog_pressure::FuelTankPressureMeasurement;
use crate::drivers::solenoid_detection::SolenoidStates;
use datatypes::units::BarG;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Watch;
use hermes_can::messages::board_status::ValveState;
use hermes_can::messages::event_messages::{
    DprState, FuelPressurization, FuelPressurizationAbort, FuelPressurizationCompleted,
};

pub struct BoardState {
    // Pressure sensors
    pub pressurization_pressure: Watch<ThreadModeRawMutex, BarG, WATCH>,
    pub fuel_tank_pressure: Watch<ThreadModeRawMutex, FuelTankPressureMeasurement, WATCH>,
    // DPR
    pub dpr_control_loop: Watch<ThreadModeRawMutex, DprState, WATCH>,
    pub dpr_pressurization: Watch<ThreadModeRawMutex, FuelPressurization, WATCH>,
    pub pressurization_info: Watch<ThreadModeRawMutex, FuelPressurizationCompleted, WATCH>,
    pub pressurization_abort: Watch<ThreadModeRawMutex, FuelPressurizationAbort, WATCH>,
    pub pressurization_kp: Mutex<ThreadModeRawMutex, f32>,
    // Valves
    pub pressurization_vent_control: Watch<ThreadModeRawMutex, ValveState, WATCH>,
    pub fuel_vent_control: Watch<ThreadModeRawMutex, ValveState, WATCH>,
    // Buzzer
    pub buzzer: Watch<ThreadModeRawMutex, BuzzerState, WATCH>,
    // Solenoid detection
    pub solenoid_states: Watch<ThreadModeRawMutex, SolenoidStates, WATCH>,
}

impl BoardState {
    const fn new() -> Self {
        Self {
            pressurization_pressure: Watch::new(),
            fuel_tank_pressure: Watch::new(),
            dpr_control_loop: Watch::new(),
            dpr_pressurization: Watch::new(),
            pressurization_info: Watch::new(),
            pressurization_abort: Watch::new(),
            pressurization_kp: Mutex::new(1.0),
            pressurization_vent_control: Watch::new(),
            fuel_vent_control: Watch::new(),
            buzzer: Watch::new(),
            solenoid_states: Watch::new(),
        }
    }
}

pub static STATE: BoardState = BoardState::new();
