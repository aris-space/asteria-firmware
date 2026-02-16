#![allow(unused_assignments)]

use crate::actuators::{CYCLE_TIME_MS, KD, KI, KP, SAFETY_LIMIT_BARG};
use crate::buzzer::{BUZZER_WATCH, BuzzerState};
use crate::drivers::WATCH;
use crate::drivers::digital_pressure::DPR_PRESSURE_WATCH;
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker, Timer, with_timeout};
use embedded_utils::fmt::warn;
use embedded_utils::trace;
use hermes_can::messages::board_status::ValveState::{Active, Inactive};
use hermes_can::messages::event_messages::DprState::{Disabled, Enabled};
use hermes_can::messages::event_messages::{
    DprState, FuelPressurization, FuelPressurizationAbort, FuelPressurizationCompleted,
};

pub static DPR_CONTROL_LOOP_WATCH: Watch<ThreadModeRawMutex, DprState, WATCH> = Watch::new();
pub static DPR_PRESSURIZATION_WATCH: Watch<ThreadModeRawMutex, FuelPressurization, WATCH> =
    Watch::new();
pub static PRESSURIZATION_INFO_WATCH: Watch<
    ThreadModeRawMutex,
    FuelPressurizationCompleted,
    WATCH,
> = Watch::new();
pub static PRESSURIZATION_ABORT_WATCH: Watch<ThreadModeRawMutex, FuelPressurizationAbort, WATCH> =
    Watch::new();

pub static PRESSURIZATION_KP: Mutex<ThreadModeRawMutex, f32> = Mutex::new(1.0);

#[embassy_executor::task]
pub(crate) async fn pid_controller(mut valve_pin: Output<'static>) {
    let mut p_watcher = DPR_PRESSURE_WATCH.receiver().unwrap();
    let mut dpr_control_loop_receiver = DPR_CONTROL_LOOP_WATCH.receiver().unwrap();
    let mut pressurization_receiver = DPR_PRESSURIZATION_WATCH.receiver().unwrap();
    let pressurization_info = PRESSURIZATION_INFO_WATCH.sender();
    let mut pressurization_abort_receiver = PRESSURIZATION_ABORT_WATCH.receiver().unwrap();

    let dpr_control_loop_sender = DPR_CONTROL_LOOP_WATCH.sender();
    let buzzer_error_sender = BUZZER_WATCH.sender();

    let mut setpoint = 0.0;
    let mut error_p = 0.0;
    let mut error_i = 0.0;
    let mut error_d = 0.0;
    let mut prev_error_p = 0.0;

    let mut loop_state = Inactive;
    let mut safety_limit_reached = false;

    let mut pressure = 0.0;
    let mut ticker = Ticker::every(Duration::from_millis(CYCLE_TIME_MS as u64));

    let pressurization_alpha = 0.1; // 5% tolerance for pressurization

    loop {
        // Check for new DPR configuration
        if let Some(cfg) = dpr_control_loop_receiver.try_changed() {
            trace!("Received new DPR config: {:?}", cfg);
            match cfg {
                Enabled(stp) => {
                    setpoint = stp;
                    loop_state = Active;
                }
                Disabled => {
                    loop_state = Inactive;
                }
            }
        }

        // Check for pressurization command
        if let Some(FuelPressurization { target_pressure }) = pressurization_receiver.try_changed()
        {
            trace!("[DPR] Starting pressurization to {} barg", target_pressure);

            // Make sure the dpr is closed before starting
            valve_pin.set_low();

            // Get the current proportional gain
            let kp = *PRESSURIZATION_KP.lock().await;

            let target_margin = (1.0 + pressurization_alpha) * target_pressure;

            // Stepwise increase pressure until target is reached or abort signal is received
            // Timeout after 15 seconds to cut sequence if target can't be reached
            let _ = with_timeout(Duration::from_secs(15), async {
                loop {
                    // Check for abort signal
                    if pressurization_abort_receiver.try_get().is_some() {
                        valve_pin.set_low();
                        Timer::after_millis(1000).await;
                        break;
                    }

                    // Read current pressure
                    let current_pressure = p_watcher.get().await;

                    // Exit if target is reached
                    if current_pressure >= target_pressure {
                        dpr_control_loop_sender.send(DprState::Enabled(target_pressure));
                        break;
                    }

                    // Proportional control for DPR open time
                    let diff = target_margin - current_pressure;
                    let open_time = (kp * diff) as u64; // in milliseconds

                    // Activate valve for calculated time
                    valve_pin.set_high();
                    Timer::after_millis(open_time).await;
                    valve_pin.set_low();

                    // Wait a bit before next iteration for pressure to stabilize
                    Timer::after_millis(500).await;
                }
            })
            .await;

            // Send completion message
            trace!("[DPR] Pressurization completed to {} barg", target_pressure);
            pressurization_info.send(FuelPressurizationCompleted);

            let _ = pressurization_receiver.try_get();
            let _ = pressurization_abort_receiver.try_get();
        }

        // Update pressure reading with available data
        pressure = p_watcher.get().await;

        // Safety check
        if pressure >= SAFETY_LIMIT_BARG {
            warn!("[DPR] Pressure limit exceeded with: {} barg", pressure);
            loop_state = Inactive;
            dpr_control_loop_sender.send(DprState::Disabled);
            safety_limit_reached = true;

            // Signal error state
            buzzer_error_sender.send(BuzzerState::Error);
        } else {
            buzzer_error_sender.send(BuzzerState::Idle);

            if safety_limit_reached {
                // Reset the flag only when pressure is back to safe levels
                safety_limit_reached = false;
                // Allow reactivation of the control loop
                loop_state = Active;
                dpr_control_loop_sender.send(DprState::Enabled(setpoint));
            }
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
