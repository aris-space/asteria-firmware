#![no_std]
#![no_main]

mod can_impl;
mod recovery_actuator_control;
mod rsbl_servo;
mod servo;
/// IN THE FINAL VERSION; MAKE SURE THAT SERVO ID 2 IS LEFT, AND SERVO ID 3 IS RIGHT POSITION!!!
mod watchdog;

use crate::can_impl::{CanReceiver, CanTransmitter, setup_can};
use crate::recovery_actuator_control::{
    ARMING_STATE, DEPLOYMENT_OCCURRED, DEPLOYMENT_SERVO_STATUS, DEPLOYMENT_TARGET_STATE,
    SEPARATION_OCCURRED, SEPARATION_SERVO_STATUS, SEPARATION_TARGET_STATE, STEERING_POWER,
    STEERING_STATUS, STEERING_TARGET_POSITIONS, ServoTargetState, SteeringStatus, WATCHDOG_STATE,
    arming_detection, deployment_task, separation_task, steering_task,
};
use crate::servo::{RecoveryActuator, Servo};
use crate::watchdog::Watchdog;
use embassy_executor::Spawner;
use embassy_futures::join::join4;
use embassy_stm32::can::CanTx;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel::{Ch1, Ch2};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::Uart;
use embassy_stm32::{bind_interrupts, can, peripherals, usart};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant};
use embassy_time::{Timer, with_timeout};
use embedded_utils::fmt::*;
use hermes_can::messages::Message;
use hermes_can::messages::board_status::{ActuatorStatus, ArmingState, WatchdogState};

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../shared/stm32g473_clocks.rs"
    ));
}

use clocks::clocks_config;

/* BEGIN CONSTANTS */

/* BEGIN MOTOR CONSTANTS */
const SAFETY_SPIRAL_POS_LEFT: i32 = -2457;
const SAFETY_SPIRAL_POS_RIGHT: i32 = 3227;
const DEPLOYMENT_INITIAL_ANGLE: f32 = 180.0;
const DEPLOYMENT_SERVO_ANGLE: f32 = 65.0;
const SEPARATION_INITIAL_ANGLE: f32 = 90.0;
const SEPARATION_SERVO_ANGLE: f32 = 180.0;
const SEP_DEPL_FREQ: Hertz = Hertz(333);
/* END MOTOR CONSTANTS */

/* BEGIN TIMER CONSTANTS */
const CAN_TX_TIMEOUT: Duration = Duration::from_millis(50);
#[allow(dead_code)]
const CAN_RX_TIMEOUT: Duration = Duration::from_millis(50);
const AUTOMATIC_SAFETY_SPIRAL_TIMER: Duration = Duration::from_millis(10000);
const STATUS_CREATION_INTERVAL: Duration = Duration::from_millis(1000);

/* END TIMER CONSTANTS */

const THIS_BOARD_ID: hermes_can::messages::BoardId = hermes_can::messages::BoardId::RecoveryBoard;
/* END CONSTANTS */

mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

// bind interrupts for UART and CAN
bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>; // data Steering Motors

    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>; // can bus
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>; // can bus
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let config = clocks_config();
    let p = embassy_stm32::init(config);

    /* Buzzer Config */
    let _buzzer = Output::new(p.PB10, Level::Low, Speed::VeryHigh);

    /* ARMING DETECTION */
    let arming_detect_pin = Input::new(p.PB11, Pull::None);
    /* BEGIN SEPARATION */
    //general power
    let sep_pwr = Output::new(p.PA1, Level::Low, Speed::Low);

    // Separation 1 control
    // setup PWM for SEP1
    // idk if this works as in the datasheet PA4 is on timer channel 2, but embassy wants it on channel 1
    let sep1_ch2_pin = PwmPin::new_ch2(p.PA4, OutputType::PushPull);
    let sep1_pwm_temp = SimplePwm::new(
        p.TIM3,
        None,
        Some(sep1_ch2_pin),
        None,
        None,
        SEP_DEPL_FREQ,
        Default::default(),
    );
    //generate sep1 servo. Giving it Pin PA4 for PWM control and PC6 for actuator detection
    let sep1 = Servo::new(sep1_pwm_temp, Ch2, Input::new(p.PC6, Pull::None));

    // Separation 2 control
    let sep2_ch1_pin = PwmPin::new_ch1(p.PA5, OutputType::PushPull);
    let sep2_pwm_temp = SimplePwm::new(
        p.TIM2,
        Some(sep2_ch1_pin),
        None,
        None,
        None,
        SEP_DEPL_FREQ,
        Default::default(),
    );
    //generate sep2 servo. Giving it Pin PA5 for PWM control and PC7 for actuator detection
    let sep2 = Servo::new(sep2_pwm_temp, Ch1, Input::new(p.PC7, Pull::None));

    let separation = RecoveryActuator::new(sep1, sep2, sep_pwr);
    /* END SEPARATION */

    /* BEGIN DEPLOYMENT */
    //general power deployment
    let depl_pwr = Output::new(p.PA2, Level::Low, Speed::Low);

    //Deployment 1 control
    let depl1_ch1_pin = PwmPin::new_ch1(p.PA6, OutputType::PushPull);
    let depl1_pwm_temp = SimplePwm::new(
        p.TIM16,
        Some(depl1_ch1_pin),
        None,
        None,
        None,
        SEP_DEPL_FREQ,
        Default::default(),
    );

    //generate depl1 servo. Giving it Pin PA6 for PWM control and PC8 for actuator detection
    let depl1 = Servo::new(depl1_pwm_temp, Ch1, Input::new(p.PC8, Pull::None));

    //Deployment 2 control
    let depl2_ch1_pin = PwmPin::new_ch1(p.PA7, OutputType::PushPull);
    let depl2_pwm_temp = SimplePwm::new(
        p.TIM17,
        Some(depl2_ch1_pin),
        None,
        None,
        None,
        SEP_DEPL_FREQ,
        Default::default(),
    );

    //generate depl2 servo. Giving it Pin PA7 for PWM control and PC9 for actuator detection
    let depl2 = Servo::new(depl2_pwm_temp, Ch1, Input::new(p.PC9, Pull::None));

    let deployment = RecoveryActuator::new(depl1, depl2, depl_pwr);
    /* END DEPLOYMENT */

    /* BEGIN STEERING MOTORS */
    //steering power
    let steer_pwr = Output::new(p.PA3, Level::Low, Speed::Low);
    //steer_pwr.set_high();
    //Timer::after_millis(200).await;

    //UART for steering motors
    //USART 1
    //UART RX: PC5
    //UART TX: PC4
    //DE Pin: PC3
    let usart_config = rsbl_servo::uart_config();
    let steering_dir = Output::new(p.PC3, Level::Low, Speed::VeryHigh);

    // this is a necessary thing: To pass the UART instance to an embassy task, the buffer here
    // needs to have static lifetime.
    static mut BUFFER_TEMP: [u8; 128] = [0; 128];
    // This unsafe block is safe because the BUFFER_TEMP array never goes out of scope (declared here in main and main never ends), is never used within the main function after declaration
    // and will be integrated into the UART only once. This assures that only a single mutable reference to the Buffer exists.
    #[allow(static_mut_refs)]
    let steering_buffer = unsafe { &mut BUFFER_TEMP };

    let steering_temp: Uart<Async> = Uart::new(
        p.USART1,
        p.PC5,
        p.PC4,
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH2,
        usart_config,
    )
    .expect("Error while configuring steering motors");

    let steering = rsbl_servo::RsblServo::new(steering_temp, steering_dir, steering_buffer);
    let steering_watchdog = Watchdog::new(AUTOMATIC_SAFETY_SPIRAL_TIMER);
    let steering_detect = Input::new(p.PA8, Pull::None);
    /* END STEERING MOTORS */

    /* BEGIN SPI FLASH */
    // SPI2 is the SPI bus for the Flash chip
    // SPI2.SCK on PB13
    // SPI2.MISO on PB14
    // SPI2.MOSI on PB15
    let flash_spi = Spi::new(
        p.SPI2,
        p.PB13,
        p.PB15,
        p.PB14,
        p.DMA2_CH5,
        p.DMA2_CH6,
        Default::default(),
    );

    // Flash chip select on PB12
    // Speed is set very high, because for each SPI transaction we toggle
    let flash_cs = Output::new(p.PB12, Level::High, Speed::VeryHigh);

    let _flash_spi =
        embedded_hal_bus::spi::ExclusiveDevice::new(flash_spi, flash_cs, embassy_time::Delay)
            .expect("Error while creating exclusive device. CS Pin set failed.");
    /* END SPI FLASH */

    /* BEGIN CAN BUS */
    // CAN FD
    // CAN.Rx is on PB8
    // CAN.Tx is on PB9
    let can = setup_can(p.FDCAN1, p.PB8, p.PB9, Irqs);
    let (can_tx, mut can_rx, _prop) = can.split();

    /* END CAN BUS */

    /* BEGIN LEDS */
    // LED1 is on PB0, is green
    // LED2 is on PB1, is yellow
    // LED3 is on PB2, is red
    let _led_green = Output::new(p.PB0, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PB1, Level::Low, Speed::Low);
    let mut led_red = Output::new(p.PB2, Level::Low, Speed::Low);
    /* END LEDS */

    debug!(
        "pkg_name: {}, git_commit_hash_short: {}, git_dirty: {}, profile: {}, features: {}, rustc: {}, target: {}",
        built_info::PKG_NAME,
        built_info::GIT_COMMIT_HASH_SHORT,
        built_info::GIT_DIRTY,
        built_info::PROFILE,
        built_info::FEATURES,
        built_info::RUSTC,
        built_info::TARGET
    );

    //indication that async is working correctly, hopefully
    spawner.spawn(blink(led_yellow)).unwrap();
    spawner
        .spawn(steering_task(
            steering,
            steer_pwr,
            steering_detect,
            steering_watchdog,
        ))
        .unwrap();
    spawner.spawn(separation_task(separation)).unwrap();
    spawner.spawn(deployment_task(deployment)).unwrap();
    spawner.spawn(can_tx_task(can_tx)).unwrap();
    spawner.spawn(arming_detection(arming_detect_pin)).unwrap();

    //now start with CAN tx stuff
    let separation_target_state_tx = SEPARATION_TARGET_STATE.sender();
    let deployment_target_state_tx = DEPLOYMENT_TARGET_STATE.sender();
    let steering_target_pos_tx = STEERING_TARGET_POSITIONS.sender();
    let steering_pwr_tx = STEERING_POWER.sender();
    loop {
        match can_rx.recv().await {
            Ok((msg, _tsp)) => {
                led_red.set_low();
                //handle received messages. They are already filtered
                match msg {
                    Message::ResetAll(_) => {
                        info!("Resetting Recovery Board");
                        cortex_m::peripheral::SCB::sys_reset();
                    }

                    Message::ResetSpecific(x) => {
                        if x.board_id == THIS_BOARD_ID {
                            info!("Resetting Recovery Board");
                            cortex_m::peripheral::SCB::sys_reset();
                        }
                    }

                    Message::UTCTimeUpdate(x) => {
                        info!("UTCTimeUpdate: {}", x);
                    }

                    Message::RecoveryPowerConfig(x) => {
                        info!("RecoveryPowerConfig: {}", x);

                        if x.steering_enabled {
                            steering_pwr_tx.send(true);
                        } else {
                            steering_pwr_tx.send(false);
                        }

                        if x.separation_enabled {
                            separation_target_state_tx.send(ServoTargetState::PoweredOn);
                        } else {
                            separation_target_state_tx.send(ServoTargetState::PoweredOff);
                        }

                        if x.deployment_enabled {
                            deployment_target_state_tx.send(ServoTargetState::PoweredOn);
                        } else {
                            deployment_target_state_tx.send(ServoTargetState::PoweredOff);
                        }
                    }

                    Message::SeparationTrigger(_) => {
                        info!("SeparationTrigger");
                        separation_target_state_tx.send(ServoTargetState::Actuated);
                    }

                    Message::DeploymentTrigger(_) => {
                        info!("DeploymentTrigger");
                        deployment_target_state_tx.send(ServoTargetState::Actuated);
                    }

                    Message::SteeringTargetPositions(x) => {
                        info!("SteeringTargetPositions: {}", x);
                        // THIS IS IMPORTANT! Positive positions from FC mean pulling line in, resulting in negative positions
                        // to the steering motors
                        match with_timeout(
                            Duration::from_millis(100),
                            steering_target_pos_tx.send([0 - x.left_pos, 0 - x.right_pos]),
                        )
                        .await
                        {
                            Ok(_) => {}
                            Err(_e) => {}
                        }
                    }

                    _ => {}
                }
            }
            Err(e) => {
                error!("HELP! THERE IS A CAN ERROR!!!! {}", e);
                error!("AAAAAAAAAAAAAAAAAHHHHHHHHHHHHHHHHHHH");
                led_red.set_high();
                Timer::after_millis(10).await;
            }
        }
    }
}

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(900).await;
    }
}

#[embassy_executor::task]
async fn can_tx_task(can_tx: CanTx<'static>) {
    let can_tx: Mutex<NoopRawMutex, _> = Mutex::new(can_tx);

    let status_creation_task = async {
        let mut last = Instant::now();
        let mut steering_status_rx = STEERING_STATUS.receiver().unwrap();
        let mut separation_status_rx = SEPARATION_SERVO_STATUS.receiver().unwrap();
        let mut deployment_status_rx = DEPLOYMENT_SERVO_STATUS.receiver().unwrap();
        let mut steering_watchdog_status_rx = WATCHDOG_STATE.receiver().unwrap();
        let mut arming_state_rx = ARMING_STATE.receiver().unwrap();

        let mut sep1_status = ActuatorStatus::default();
        let mut sep2_status = ActuatorStatus::default();
        let mut depl1_status = ActuatorStatus::default();
        let mut depl2_status = ActuatorStatus::default();
        let mut steering_general = ActuatorStatus::default();
        let mut steering_left_connected = false;
        let mut steering_right_connected = false;
        let mut steering_watchdog_status = WatchdogState::default();
        let mut arming_state = ArmingState::default();
        let mut common = hermes_can::messages::board_status::StatusCommonMessage::default();

        loop {
            if last + STATUS_CREATION_INTERVAL <= Instant::now() {
                last = Instant::now();
                //try to update separation status
                if let Some(data) = separation_status_rx.try_changed() {
                    [sep1_status, sep2_status] = data;
                }
                //try to update deployment status
                if let Some(data) = deployment_status_rx.try_changed() {
                    [depl1_status, depl2_status] = data;
                }
                //try to update steering status
                if let Some(data) = steering_status_rx.try_changed() {
                    match data {
                        SteeringStatus::NotConnected => {
                            steering_general = ActuatorStatus::NotConnected;
                            steering_left_connected = false;
                            steering_right_connected = false;
                        }
                        SteeringStatus::Connected => {
                            steering_general = ActuatorStatus::Connected;
                            steering_left_connected = false;
                            steering_right_connected = false;
                        }
                        SteeringStatus::Responsive(values) => {
                            steering_general = ActuatorStatus::PowerOn;
                            // update left response bool by reading if it has responded with data
                            match values[0] {
                                Some(_) => {
                                    steering_left_connected = true;
                                }
                                None => {
                                    steering_left_connected = false;
                                }
                            }
                            // update right response bool by reading if it has responded with data
                            match values[1] {
                                Some(_) => {
                                    steering_right_connected = true;
                                }
                                None => {
                                    steering_right_connected = false;
                                }
                            }
                        }
                    }
                }
                //try to update watchdog status
                if let Some(data) = steering_watchdog_status_rx.try_changed() {
                    steering_watchdog_status = data;
                }

                if let Some(data) = arming_state_rx.try_changed() {
                    arming_state = data;
                }

                common.micros_since_restart = Instant::as_micros(&Instant::now());

                //now actually construct the REC board status message with the data collected
                let msg = hermes_can::messages::board_status::RecoveryBoardStatus {
                    common: common.clone(),
                    sep1_status: sep1_status.clone(),
                    sep2_status: sep2_status.clone(),
                    depl1_status: depl1_status.clone(),
                    depl2_status: depl2_status.clone(),
                    steering_general: steering_general.clone(),
                    steering_left_connected,
                    steering_right_connected,
                    steering_watchdog_status: steering_watchdog_status.clone(),
                    arming_state: arming_state.clone(),
                };
                info!("status: {}", msg);
                let mut tx = can_tx.lock().await;
                match with_timeout(CAN_TX_TIMEOUT, tx.transmit(msg)).await {
                    Ok(Ok(_)) => {
                        trace!("sent REC Board status message");
                    }
                    Ok(Err(err)) => {
                        error!("CAN TX error: {:?}", err);
                    }
                    Err(_) => {
                        error!("CAN TX timed out after {} ms", CAN_TX_TIMEOUT);
                    }
                }
                drop(tx);
            } else {
                Timer::after_millis(25).await;
            }
        }
    };

    let steering_sending_task = async {
        let mut steering_status_rx = STEERING_STATUS.receiver().unwrap();
        loop {
            let data = steering_status_rx.changed().await;
            #[allow(clippy::single_match)]
            match data {
                // only send sth when I have actual data to send, otherwise don't even bother
                SteeringStatus::Responsive(data) => {
                    let mut positions: [i32; 2] = [0; 2];

                    // here is the same issue as with the sending things. The FC positions are inverted from the
                    // positions on the REC board
                    if let Some(left_data) = data[0] {
                        positions[0] = -left_data.angle;
                    };
                    if let Some(right_data) = data[1] {
                        positions[1] = -right_data.angle;
                    };

                    let msg = hermes_can::messages::event_messages::SteeringActualPositions {
                        left_pos: positions[0],
                        right_pos: positions[1],
                    };
                    info!("msg: {}", msg);
                    let mut tx = can_tx.lock().await;
                    match with_timeout(CAN_TX_TIMEOUT, tx.transmit(msg)).await {
                        Ok(Ok(_)) => {
                            trace!("sent steering actual positions");
                        }
                        Ok(Err(err)) => {
                            error!("CAN TX error: {:?}", err);
                        }
                        Err(_) => {
                            error!("CAN TX timed out after {} ms", CAN_TX_TIMEOUT);
                        }
                    }
                    drop(tx);
                }
                _ => {}
            }
        }
    };

    let separation_response_task = async {
        let mut separation_triggered_rx = SEPARATION_OCCURRED.receiver().unwrap();
        loop {
            let rx = separation_triggered_rx.changed().await;
            if rx {
                let msg = hermes_can::messages::event_messages::SeparationOccurred {};
                let mut tx = can_tx.lock().await;
                match with_timeout(CAN_TX_TIMEOUT, tx.transmit(msg)).await {
                    Ok(Ok(_)) => {
                        trace!("sent Separation Occurred");
                    }
                    Ok(Err(err)) => {
                        error!("CAN TX error: {:?}", err);
                    }
                    Err(_) => {
                        error!("CAN TX timed out after {} ms", CAN_TX_TIMEOUT);
                    }
                }
                drop(tx);
            }
        }
    };

    let deployment_response_task = async {
        let mut deployment_triggered_rx = DEPLOYMENT_OCCURRED.receiver().unwrap();
        loop {
            let rx = deployment_triggered_rx.changed().await;
            if rx {
                let msg = hermes_can::messages::event_messages::DeploymentOccurred {};
                let mut tx = can_tx.lock().await;
                match with_timeout(CAN_TX_TIMEOUT, tx.transmit(msg)).await {
                    Ok(Ok(_)) => {
                        trace!("sent Deployment Occurred");
                    }
                    Ok(Err(err)) => {
                        error!("CAN TX error: {:?}", err);
                    }
                    Err(_) => {
                        error!("CAN TX timed out after {} ms", CAN_TX_TIMEOUT);
                    }
                }
                drop(tx);
            }
        }
    };

    join4(
        status_creation_task,
        deployment_response_task,
        steering_sending_task,
        separation_response_task,
    )
    .await;
}
