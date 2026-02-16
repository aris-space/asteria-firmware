#![allow(clippy::single_match)]
#![allow(clippy::collapsible_else_if)]
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::SeqCst;
use embassy_futures::join::join;
use embassy_stm32::gpio::{Input, Level, Output};
use embassy_stm32::peripherals::{TIM16, TIM17, TIM2, TIM3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::watch::Watch;
use embassy_time::{with_timeout, Timer};
use embassy_time::{Duration, Instant};
use embedded_utils::fmt::*;
// this is maybe not nice, think about using another enum?
use crate::recovery_actuator_control::SteeringStatus::{Connected, NotConnected, Responsive};
use crate::rsbl_servo::{RsblData, LEFT, RIGHT};
use crate::servo::RecoveryActuator;
use crate::{
    rsbl_servo, watchdog, DEPLOYMENT_INITIAL_ANGLE, DEPLOYMENT_SERVO_ANGLE, SAFETY_SPIRAL_POS_LEFT,
    SAFETY_SPIRAL_POS_RIGHT, SEPARATION_INITIAL_ANGLE, SEPARATION_SERVO_ANGLE,
};
use hermes_can::messages::board_status::{ActuatorStatus, ArmingState, WatchdogState};

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
    /// no current sink is detected at the actuators
    NotConnected,
    /// This state is only possible before power is on
    Connected,

    /// in this array is saved the Motor data from the REC board. In data0 is the array containing the left data,
    /// while in data1 is the array containing the right data. The data is wrapped in an option to indicate if
    /// any data could be read or not
    Responsive([Option<RsblData>; 2]),
}
/// watch for giving steering target positions to steering_task
pub static STEERING_TARGET_POSITIONS: Channel<CriticalSectionRawMutex, [i32; 2], 3> =
    Channel::new();

/// watch for setting steering power
pub static STEERING_POWER: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();

/// status that also includes the data read from the motors
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
    steering_actuator_detect: Input<'static>,
    mut watchdog: watchdog::Watchdog,
) {
    let motor_targets_rx = STEERING_TARGET_POSITIONS.receiver();
    let mut motor_power = STEERING_POWER.receiver().unwrap();
    let steering_status = STEERING_STATUS.sender();
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
                    match motor_targets_rx.try_receive() {
                        Ok(target_positions) => {
                            //on first value reception, activate watchdog
                            if !watchdog_active {
                                watchdog.start();
                                watchdog_active = true;
                                watchdog_state_tx.send(WatchdogState::Active);
                            }
                            // pet the watchdog
                            watchdog.update();
                            // set target positions to steering
                            match steering
                                .steer_parachutes(target_positions[0], target_positions[1])
                                .await
                            {
                                Ok(()) => {}
                                Err(e) => {
                                    error!("Error in steering parachutes: {:?}", e)
                                }
                            }
                        }
                        Err(_) => {}
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

                let mut angles = [None, None];
                // read out position data for both servos roughly every 100 ms
                if Instant::now() - last >= Duration::from_millis(100) {
                    last = Instant::now();
                    match steering.read_steering_data(LEFT).await {
                        Ok(left_val) => {
                            angles[0] = left_val;
                        }
                        Err(e) => {
                            error!("Error in reading left steering data: {:?}", e)
                        }
                    }
                    match steering.read_steering_data(RIGHT).await {
                        Ok(right_val) => {
                            angles[1] = right_val;
                        }
                        Err(e) => {
                            error!("Error in reading right steering data: {:?}", e)
                        }
                    }
                    steering_status.send(Responsive(angles));
                }
            } else {
                // deactivate steering, make sure to wait a bit...
                active = false;
                safety_spiral_active = false;
                watchdog_active = false;

                // get the current state of the steering motor connection
                steering_status.send(match steering_actuator_detect.get_level() {
                    Level::High => Connected,
                    Level::Low => NotConnected,
                });
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
        match with_timeout(Duration::from_millis(1000), target_rx.changed()).await {
            Ok(data) => {
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
            Err(_) => {}
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

pub static ARMING_STATE: Watch<CriticalSectionRawMutex, ArmingState, 1> = Watch::new();
#[embassy_executor::task]
pub async fn arming_detection(arming_detect: Input<'static>) {
    let arming_sender = ARMING_STATE.sender();
    loop {
        // arming is high if safed, and low if armed
        if arming_detect.is_low() {
            arming_sender.send(ArmingState::Armed);
        } else {
            arming_sender.send(ArmingState::Safe);
        }

        Timer::after_millis(1000).await;
    }
}
