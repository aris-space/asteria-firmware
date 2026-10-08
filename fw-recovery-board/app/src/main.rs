// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Firmware for the ASTERIA Recovery Board

#![no_std]
#![no_main]

mod actuator_control;
mod build_info;
mod can_io;
mod cats;
mod rsbl_servo;
mod servo;
mod watchdog;

use crate::actuator_control::{DEPLOYMENT_TARGET_STATE, SEPARATION_TARGET_STATE, ServoTargetState};
use crate::can_io::{INPUTS, OUTPUTS, ReceivedMessage};
use crate::servo::{RecoveryActuator, Servo};
use crate::watchdog::Watchdog;
use can_utils::broadcast::Broadcast;
use can_utils::collector::Collector as _;
use can_utils::rxtx::TypedCanReceive as _;
use can_utils::setup::{make_multiplexable, setup_can};
use data_core::can::hal::CanDecode;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel::{Ch1, Ch2};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::Uart;
use embassy_stm32::{bind_interrupts, can, dma, peripherals, usart};
use embassy_time::Duration;
use embassy_time::Timer;
use embedded_utils::fmt::*;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32g473_clocks.rs"
    ));
}

use clocks::clocks_config;

/* BEGIN CONSTANTS */

/* BEGIN MOTOR CONSTANTS */
const SAFETY_SPIRAL_POS_LEFT: i32 = -2457;
const SAFETY_SPIRAL_POS_RIGHT: i32 = 3227;
const DEPLOYMENT_INITIAL_ANGLE: f32 = 180.0;
const DEPLOYMENT_SERVO_ANGLE: f32 = 65.0;
const SEPARATION_INITIAL_ANGLE: f32 = 180.0;
const SEPARATION_SERVO_ANGLE: f32 = 0.0;
const SEP_DEPL_FREQ: Hertz = Hertz(333);
/* END MOTOR CONSTANTS */

/* BEGIN TIMER CONSTANTS */
pub const CAN_TX_TIMEOUT: Duration = Duration::from_millis(50);
pub const STATUS_CREATION_INTERVAL: Duration = Duration::from_millis(1000);
const AUTOMATIC_SAFETY_SPIRAL_TIMER: Duration = Duration::from_millis(10000);

/* END TIMER CONSTANTS */

const THIS_BOARD_ID: datatypes::status::BoardId = datatypes::status::BoardId::RecoveryBoard;
/* END CONSTANTS */

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

// bind interrupts for UART and CAN
bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>; // steering motors (RS-485)

    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>; // can bus
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>; // can bus

    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    DMA2_CHANNEL5 => dma::InterruptHandler<peripherals::DMA2_CH5>;
    DMA2_CHANNEL6 => dma::InterruptHandler<peripherals::DMA2_CH6>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let config = clocks_config();
    let p = embassy_stm32::init(config);

    /* Buzzer Config */
    let _buzzer = Output::new(p.PB10, Level::Low, Speed::VeryHigh);

    /* ARMING DETECTION */
    let arming_detect_pin = Input::new(p.PB11, Pull::None);
    /* CATS backup flight computer, pulled up and driven low once it triggered */
    let cats_separation = Input::new(p.PA8, Pull::Up); // CATS IO 1
    let cats_deployment = Input::new(p.PA9, Pull::Up); // CATS IO 2

    /* BEGIN SEPARATION */
    // separation power trigger
    let sep_pwr = Output::new(p.PC0, Level::Low, Speed::Low);

    // Separation 1 control
    // setup PWM for SEP1
    let sep1_ch2_pin = PwmPin::new(p.PA4, OutputType::PushPull);
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
    let sep2_ch1_pin = PwmPin::new(p.PA5, OutputType::PushPull);
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
    // deployment power trigger
    let depl_pwr = Output::new(p.PC1, Level::Low, Speed::Low);

    //Deployment 1 control
    let depl1_ch1_pin = PwmPin::new(p.PA6, OutputType::PushPull);
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
    let depl2_ch1_pin = PwmPin::new(p.PA7, OutputType::PushPull);
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
    // steering power trigger
    let steer_pwr = Output::new(p.PC2, Level::Low, Speed::Low);

    // RS-485 on USART2 with hardware driver enable
    // USART2.DE on PA1
    // USART2.TX on PA2
    // USART2.RX on PA3
    let usart_config = rsbl_servo::uart_config();

    // SAFETY:
    // The main function is only called once, thus
    // the static is narrowly scoped and will be integrated into the UART only once.
    // This assures that only a single mutable reference to the Buffer exists.
    let steering_buffer = unsafe {
        static mut BUFFER_TEMP: [u8; 128] = [0; 128];
        #[allow(static_mut_refs)]
        &mut BUFFER_TEMP
    };

    let steering_uart: Uart<Async> = Uart::new_with_de(
        p.USART2,
        p.PA3,
        p.PA2,
        p.PA1,
        p.DMA1_CH1,
        p.DMA1_CH2,
        Irqs,
        usart_config,
    )
    .unwrap();

    let steering = rsbl_servo::RsblServo::new(steering_uart, steering_buffer);
    let steering_watchdog = Watchdog::new(AUTOMATIC_SAFETY_SPIRAL_TIMER);
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
        Irqs,
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
    let can = setup_can(p.FDCAN1, p.PB8, p.PB9, Irqs, ReceivedMessage::SUPPORTED_IDS);
    let (can_tx, mut can_rx, _prop) = can.split();
    let can_tx = make_multiplexable(can_tx);
    /* END CAN BUS */

    /* BEGIN LEDS */
    // LED1 is on PB0, is green
    // LED2 is on PB1, is yellow
    // LED3 is on PB2, is red
    let led_green = Output::new(p.PB0, Level::Low, Speed::Low);
    let mut led_yellow = Output::new(p.PB1, Level::Low, Speed::Low);
    let led_red = Output::new(p.PB2, Level::Low, Speed::Low);
    /* END LEDS */

    debug!("build info: {:?}", build_info::BUILD_INFO.get());
    OUTPUTS
        .build_info
        .sender()
        .send(crate::build_info::BUILD_INFO.get().clone());

    spawner.spawn(heartbeat(led_green).unwrap());
    spawner.spawn(build_info::build_status_blinky(led_red).unwrap());
    spawner.spawn(actuator_control::steering_task(steering, steer_pwr, steering_watchdog).unwrap());
    spawner.spawn(actuator_control::separation_task(separation).unwrap());
    spawner.spawn(actuator_control::deployment_task(deployment).unwrap());
    spawner.spawn(actuator_control::arming_detection_task(arming_detect_pin).unwrap());
    spawner.spawn(cats::cats_task(cats_separation, cats_deployment).unwrap());
    spawner.spawn(can_io::can_tx_task(can_tx).unwrap());
    OUTPUTS.start_broadcasting(spawner, can_tx).unwrap();

    //now start with CAN tx stuff
    let separation_target_state_tx = SEPARATION_TARGET_STATE.sender();
    let deployment_target_state_tx = DEPLOYMENT_TARGET_STATE.sender();
    let steering_pwr_tx = INPUTS.steering_power.sender();
    loop {
        match can_rx.recv().await {
            Ok(msg) => {
                led_yellow.set_low();
                //handle received messages. They are already filtered
                match msg {
                    ReceivedMessage::ResetAll(_) => {
                        info!("Resetting Recovery Board");
                        cortex_m::peripheral::SCB::sys_reset();
                    }

                    ReceivedMessage::ResetSpecific(board) => {
                        if board == THIS_BOARD_ID {
                            info!("Resetting Recovery Board");
                            cortex_m::peripheral::SCB::sys_reset();
                        }
                    }

                    ReceivedMessage::UTCTimeUpdate(x) => {
                        info!("UTCTimeUpdate: {}", x);
                    }

                    ReceivedMessage::RecoveryPowerConfig(x) => {
                        info!("RecoveryPowerConfig: {}", x);

                        steering_pwr_tx.send(x.steering_enabled);
                        separation_target_state_tx
                            .send(ServoTargetState::powered(x.separation_enabled));
                        deployment_target_state_tx
                            .send(ServoTargetState::powered(x.deployment_enabled));
                    }

                    ReceivedMessage::SeparationTrigger(_) => {
                        info!("SeparationTrigger");
                        separation_target_state_tx.send(ServoTargetState::Actuated);
                    }

                    ReceivedMessage::DeploymentTrigger(_) => {
                        info!("DeploymentTrigger");
                        deployment_target_state_tx.send(ServoTargetState::Actuated);
                    }
                    msg => {
                        // update the collected inputs and ignore if the message is irrelevant
                        let _ = INPUTS.update_from(msg);
                    }
                }
            }
            Err(e) => {
                error!("HELP! THERE IS A CAN ERROR!!!! {}", e);
                led_yellow.set_high();
                Timer::after_millis(10).await;
            }
        }
    }
}

/// Green LED heartbeat, 1 Hz at 50% duty cycle
#[embassy_executor::task]
async fn heartbeat(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(500).await;
    }
}
