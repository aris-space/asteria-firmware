//! Module to talk to the steering, deployment and separation motors.

use crate::actuator_control::SteeringStatus::{Responsive, Unpowered};
use crate::can_io::{INPUTS, OUTPUTS};
use crate::rsbl_servo::{LEFT, RIGHT};
use crate::servo::RecoveryActuator;
use crate::{
    DEPLOYMENT_INITIAL_ANGLE, DEPLOYMENT_SERVO_ANGLE, SAFETY_SPIRAL_POS_LEFT,
    SAFETY_SPIRAL_POS_RIGHT, SEPARATION_INITIAL_ANGLE, SEPARATION_SERVO_ANGLE, rsbl_servo,
    watchdog,
};
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::SeqCst;
use datatypes::status::ArmingState;
use dp_recovery_board::{ActuatorStatus, SteeringPositions, WatchdogState};
use embassy_futures::join::join;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::peripherals::{TIM2, TIM3, TIM16, TIM17};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant};
use embassy_time::{Timer, with_timeout};
use embedded_utils::fmt::*;

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

#[derive(Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SteeringStatus {
    #[default]
    /// steering is unpowered, so the motors cannot be detected
    Unpowered,

    /// Indicates if data from the motors could be read or not for [left, right].
    Responsive([bool; 2]),
}

/// status that of the motors
pub static STEERING_STATUS: Watch<CriticalSectionRawMutex, SteeringStatus, 2> = Watch::new();

/// indicator for power for steering (setting target positions etc. just won't do anything if this is not true)
/// This is also purely internal for this file only
static STEERING_POWER_STATUS: AtomicBool = AtomicBool::new(false);

/// indicator for steering watchdog state
pub static WATCHDOG_STATE: Watch<CriticalSectionRawMutex, WatchdogState, 1> = Watch::new();

#[embassy_executor::task]
pub async fn steering_task(
    mut steering: rsbl_servo::RsblServo<'static>,
    mut pwr: Output<'static>,
    mut watchdog: watchdog::Watchdog,
) {
    let mut motor_targets_rx = INPUTS.steering_target_positions.receiver().unwrap();
    let mut motor_power = INPUTS.steering_power.receiver().unwrap();
    let steering_status = STEERING_STATUS.sender();
    let steering_positions = OUTPUTS.steering_actual_positions.sender();
    let watchdog_state_tx = WATCHDOG_STATE.sender();

    let power_task = async {
        loop {
            match motor_power.changed().await {
                true => {
                    pwr.set_high();
                    // wait a bit for motor MCUs to start up before giving them any instructions
                    Timer::after_millis(1000).await;
                    STEERING_POWER_STATUS.store(true, SeqCst);
                }
                false => {
                    STEERING_POWER_STATUS.store(false, SeqCst);
                    pwr.set_low();
                }
            }
        }
    };

    // set steering positions and try to get the current steering data
    let steering_task = async {
        let mut last = Instant::now();
        let mut active: bool = false;
        let mut watchdog_active: bool = false;
        let mut safety_spiral_active: bool = false;
        loop {
            // detect if steering is active
            if STEERING_POWER_STATUS.load(SeqCst) {
                //on first startup execute the startup sequence
                if !active {
                    //activate steering, set up UART Bus anew, lock motors
                    match steering.startup_sequence().await {
                        Ok(()) => {}
                        Err(e) => {
                            error!("Error in steering startup: {:?}", e);
                        }
                    }
                    active = true;
                }

                // check that the watchdog is still active
                if watchdog.check() {
                    // check if new values are available
                    if let Some(target_positions) = motor_targets_rx.try_changed() {
                        //on first value reception, activate watchdog
                        if !watchdog_active {
                            watchdog.start();
                            watchdog_active = true;
                            watchdog_state_tx.send(WatchdogState::Active);
                        }
                        // pet the watchdog
                        watchdog.update();
                        // set target positions to steering (flip as motors are counting revolutions the other way around)
                        match steering
                            .steer_parachutes(
                                -target_positions.left_pos,
                                -target_positions.right_pos,
                            )
                            .await
                        {
                            Ok(()) => {}
                            Err(e) => {
                                error!("Error in steering parachutes: {:?}", e)
                            }
                        }
                    }
                } else {
                    if !safety_spiral_active {
                        watchdog_state_tx.send(WatchdogState::Expired);
                        // watchdog has expired, set safety spiral positions
                        match steering
                            .steer_parachutes(SAFETY_SPIRAL_POS_LEFT, SAFETY_SPIRAL_POS_RIGHT)
                            .await
                        {
                            Ok(()) => {
                                warn!("Safety spiral watchdog triggered");
                                safety_spiral_active = true;
                            }
                            Err(e) => {
                                error!("Error in steering enabling safety spiral: {:?}", e)
                            }
                        }
                    }
                }

                let mut positions = SteeringPositions::default();
                let mut connectedness = [false; 2];
                // read out position data for both servos roughly every 100 ms
                if Instant::now() - last >= Duration::from_millis(100) {
                    last = Instant::now();
                    match steering.read_steering_data(LEFT).await {
                        Ok(left_val) => {
                            // SteeringPositions is flipped from what the driver outputs.
                            positions.left_pos = -left_val.map(|d| d.angle).unwrap_or_default();
                            connectedness[0] = left_val.is_some();
                        }
                        Err(e) => {
                            error!("Error in reading left steering data: {:?}", e)
                        }
                    }
                    match steering.read_steering_data(RIGHT).await {
                        Ok(right_val) => {
                            // SteeringPositions is flipped from what the driver outputs.
                            positions.right_pos = -right_val.map(|d| d.angle).unwrap_or_default();
                            connectedness[1] = right_val.is_some();
                        }
                        Err(e) => {
                            error!("Error in reading right steering data: {:?}", e)
                        }
                    }
                    steering_positions.send(if connectedness[0] && connectedness[1] {
                        Some(positions)
                    } else {
                        None
                    });
                    steering_status.send(Responsive(connectedness));
                }
            } else {
                // deactivate steering, make sure to wait a bit...
                active = false;
                safety_spiral_active = false;
                watchdog_active = false;

                // without power the motors cannot be detected
                steering_status.send(Unpowered);
            }
            // delay a bit before next iteration through this loop
            Timer::after_millis(10).await;
        }
    };

    join(power_task, steering_task).await;
}

#[derive(Clone, Copy, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ServoTargetState {
    #[default]
    PoweredOff,
    PoweredOn,
    Actuated,
}

impl ServoTargetState {
    /// the resting state for a power-enable flag from the power config message
    pub fn powered(enabled: bool) -> Self {
        if enabled {
            Self::PoweredOn
        } else {
            Self::PoweredOff
        }
    }
}

/// receive target states for separation Actuators
pub static SEPARATION_TARGET_STATE: Watch<CriticalSectionRawMutex, ServoTargetState, 1> =
    Watch::new();

/// send ActuatorStatus for REC board status messages. index 0 is sep1, index 1 is sep2.
pub static SEPARATION_SERVO_STATUS: Watch<CriticalSectionRawMutex, [ActuatorStatus; 2], 1> =
    Watch::new();

/// set high when separation occurred
pub static SEPARATION_OCCURRED: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();

/// this is the task for executing separation. It can currently only accept the RecoveryActuator struct as it
/// is defined in main, due to the specific implementation of all that stuff
#[embassy_executor::task]
pub async fn separation_task(mut separation: RecoveryActuator<TIM3, TIM2>) {
    let mut target_rx = SEPARATION_TARGET_STATE.receiver().unwrap();
    let status_tx = SEPARATION_SERVO_STATUS.sender();
    let separation_flag_tx = SEPARATION_OCCURRED.sender();

    loop {
        // wait for TargetState to be provided by CAN message
        if let Ok(data) = with_timeout(Duration::from_millis(1000), target_rx.changed()).await {
            match data {
                ServoTargetState::PoweredOff => {
                    separation.deactivate_servo();
                }
                ServoTargetState::PoweredOn => {
                    separation.activate_servo();
                    let _ = separation.set_angle(SEPARATION_INITIAL_ANGLE);
                }
                ServoTargetState::Actuated => {
                    separation.activate_servo();
                    // we might want to change this to a wiggle function
                    let _ = separation.set_angle(SEPARATION_SERVO_ANGLE);
                    // still needs to send separation occurred when this is done
                    separation_flag_tx.send(true);
                }
            }
        }
        // update actuator connection thingy
        status_tx.send(separation.get_actuator_status());
    }
}

/// target state for deployment, same as for separation
pub static DEPLOYMENT_TARGET_STATE: Watch<CriticalSectionRawMutex, ServoTargetState, 1> =
    Watch::new();

/// state of the deployment servos, same as for separation
pub static DEPLOYMENT_SERVO_STATUS: Watch<CriticalSectionRawMutex, [ActuatorStatus; 2], 1> =
    Watch::new();

/// set high when deployment occurred
pub static DEPLOYMENT_OCCURRED: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();

#[embassy_executor::task]
pub async fn deployment_task(mut deployment: RecoveryActuator<TIM16, TIM17>) {
    let mut target_rx = DEPLOYMENT_TARGET_STATE.receiver().unwrap();
    let status_tx = DEPLOYMENT_SERVO_STATUS.sender();
    let deployment_status_tx = DEPLOYMENT_OCCURRED.sender();

    loop {
        // wait for TargetState to be provided by CAN message
        match with_timeout(Duration::from_millis(1000), target_rx.changed()).await {
            Ok(data) => {
                match data {
                    ServoTargetState::PoweredOff => {
                        deployment.deactivate_servo();
                    }
                    ServoTargetState::PoweredOn => {
                        deployment.activate_servo();
                        let _ = deployment.set_angle(DEPLOYMENT_INITIAL_ANGLE);
                    }
                    ServoTargetState::Actuated => {
                        deployment.activate_servo();
                        // we might want to change this to a wiggle function
                        let _ = deployment.set_angle(DEPLOYMENT_SERVO_ANGLE);
                        // still needs to send deployment occurred when this is done
                        deployment_status_tx.send(true);
                        Timer::after_millis(30000).await;
                        let _ = deployment.set_angle(DEPLOYMENT_INITIAL_ANGLE);
                    }
                }
            }
            Err(_e) => {}
        }
        // update actuator connection thingy
        status_tx.send(deployment.get_actuator_status());
    }
}

pub static ARMING_STATE: Watch<CriticalSectionRawMutex, ArmingState, 2> = Watch::new();

/// Poll interval for the Arming pin (10 Hz)
const ARMING_POLL_MS: u64 = 100;

#[embassy_executor::task]
pub async fn arming_detection_task(arming_detect: Input<'static>) {
    let arming_sender = ARMING_STATE.sender();
    loop {
        // arming is high if safed, and low if armed
        if arming_detect.is_low() {
            arming_sender.send(ArmingState::Armed);
        } else {
            arming_sender.send(ArmingState::Safe);
        }

        Timer::after_millis(ARMING_POLL_MS).await;
    }
}
