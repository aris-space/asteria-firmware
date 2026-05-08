#![allow(unused_assignments)]

use defmt::println;
use crate::actuators::{CYCLE_TIME_MS, KD, KI, KP, SAFETY_LIMIT_BARG};
use crate::buzzer::BuzzerState;
use crate::globals::STATE;
use datatypes::actuator::DPRValve;
use datatypes::status::ValveState::{Active, Inactive};
use embassy_stm32::gpio::Output;
use embassy_stm32::usb::In;
use embassy_time::{Duration, Ticker};
use embedded_utils::fmt::warn;
use embedded_utils::{error, info, trace};

#[embassy_executor::task]
pub(crate) async fn pid_controller(mut valve_pin: Output<'static>) {
    let mut p_watcher = STATE.fuel_tank_pressure.receiver().unwrap();
    let mut dpr_control_loop_receiver = STATE.dpr_control_loop.receiver().unwrap();

    let dpr_control_loop_sender = STATE.dpr_control_loop.sender();
    let buzzer_error_sender = STATE.buzzer.sender();

    let mut setpoint = 0.0;
    let mut error_p = 0.0;
    let mut error_i = 0.0;
    let mut error_d = 0.0;
    let mut prev_error_p = 0.0;

    let mut loop_state = Inactive;
    let mut safety_limit_reached = false;

    let mut pressure = 0.0;
    let mut ticker = Ticker::every(Duration::from_millis(CYCLE_TIME_MS as u64));

    loop {
        // Check for new DPR configuration
        if let Some(cfg) = dpr_control_loop_receiver.try_changed() {
            match cfg {
                DPRValve::Enabled { setpoint: stp } => {
                    setpoint = stp;
                    loop_state = Active;
                }
                DPRValve::Disabled => {
                    loop_state = Inactive;
                }
            }
        }

        // Update pressure reading with available data
        let new_pressure = p_watcher.get().await.fuel_tank_pressure_filtered.0;
        if new_pressure != f32::INFINITY {
            pressure = new_pressure
        }

        // Safety check
        // ToDo: implement correctly ask lennard he will yap about it
        if pressure >= SAFETY_LIMIT_BARG {
            error!("[DPR] Pressure limit exceeded with: {} barg", pressure);
            loop_state = Inactive;
            safety_limit_reached = true;
            dpr_control_loop_sender.send(DPRValve::Disabled);

            // Signal error state
            buzzer_error_sender.send(BuzzerState::Error);
        } else {
            buzzer_error_sender.send(BuzzerState::Idle);
        }

        // PID Control
        if loop_state == Active {
            error_p = setpoint - pressure;
            error_i += error_p * CYCLE_TIME_MS;
            error_d = (error_p - prev_error_p) / CYCLE_TIME_MS;
            prev_error_p = error_p;
            let error = KP * error_p + KI * error_i + KD * error_d;

            // Actuate valve based on error
            // Mapped to Bang Bang controller
            if error > 0.0 {
                valve_pin.set_high();
            } else {
                valve_pin.set_low();
            }
        } else {
            // Ensure valve is closed when inactive
            valve_pin.set_low();
            error_i = 0.0; // Reset integral error when loop is inactive
        }
        // Wait for next cycle
        ticker.next().await;
    }
}
