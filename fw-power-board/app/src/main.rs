// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

mod blink;
mod board;
mod build_info;
mod buzzer;
mod can;
mod can_io;
mod power_indicators;
mod sensor_readout;
mod unix_time;

use crate::can::{OUTPUTS, THIS_BOARD_ID, can_board_status_task};
use crate::can_io::ReceivedMessage;
use crate::unix_time::init_utc_clock;
use board::{INA232_I2C_ADDR, Irqs};
use can_utils::broadcast::Broadcast;
use can_utils::rxtx::{RxError, TypedCanReceive};
use can_utils::setup::{make_multiplexable, setup_can};
use cortex_m::peripheral::SCB;
use data_core::can::hal::CanDecode;
use datatypes::units::InstantUs;
use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::gpio::{Input, Level, Output, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::{Duration, Instant, Timer};
use ina232::Ina232;

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
pub use ::panic_reset as _;

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32g473_clocks.rs"
    ));
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    // Initialize clocks and peripherals
    let config = clocks::clocks_config();
    let p = embassy_stm32::init(config);

    // Configure active power-rail detection (pull down is external)
    let bat_p = Input::new(p.PA0, embassy_stm32::gpio::Pull::None);
    let ext_p = Input::new(p.PA1, embassy_stm32::gpio::Pull::None);

    // Configure LEDs as outputs (low = off)
    let _led_green = Output::new(p.PA3, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PA4, Level::Low, Speed::Low);
    let _led_red = Output::new(p.PA5, Level::Low, Speed::Low);
    let led_bat_p = Output::new(p.PA6, Level::Low, Speed::Low);
    let led_ext_p = Output::new(p.PA7, Level::Low, Speed::Low);

    // Shared I2C configuration: 5 ms timeout, 400 kHz speed
    let mut shared_i2c_config = i2c::Config::default();
    shared_i2c_config.timeout = Duration::from_millis(5);
    shared_i2c_config.frequency = Hertz(400_000);

    // Initialize I2C3 (SDA = PC9, SCL = PC8) for the 5V sensor
    let i2c3 = I2c::new(
        p.I2C3,
        p.PC8, // SCL
        p.PC9, // SDA
        p.DMA2_CH3,
        p.DMA2_CH4,
        Irqs,
        shared_i2c_config,
    );
    let mut ina_rail_5v = Ina232::new_i2c(i2c3, INA232_I2C_ADDR);
    ina_rail_5v
        .init()
        .await
        .expect("Failed to init INA232 5V rail");

    // Initialize I2C2 (SDA = PA8, SCL = PA9) for the 24V sensor
    let i2c2 = I2c::new(
        p.I2C2,
        p.PA9, // SCL
        p.PA8, // SDA
        p.DMA2_CH1,
        p.DMA2_CH2,
        Irqs,
        shared_i2c_config,
    );
    let mut ina_rail_24v = Ina232::new_i2c(i2c2, INA232_I2C_ADDR);
    ina_rail_24v
        .init()
        .await
        .expect("Failed to init INA232 24V rail");

    // Initialize PWM (TIM2 CH3 on PB10) for buzzer (~3 kHz, 50% duty)
    let buzzer_pin = PwmPin::new(p.PB10, OutputType::PushPull);
    let pwm = SimplePwm::new(
        p.TIM2,
        None,                        // CH1 unused
        None,                        // CH2 unused
        Some(buzzer_pin),            // CH3 = PB10
        None,                        // CH4 unused
        Hertz(3_000),                // ~3 kHz
        CountingMode::EdgeAlignedUp, // count-up edge-aligned mode
    );

    // Initialize the CAN peripheral (FDCAN1 with pins PB8=RX, PB9=TX) and split TX/RX
    let can = setup_can(p.FDCAN1, p.PB8, p.PB9, Irqs, ReceivedMessage::SUPPORTED_IDS);
    let (can_tx, mut can_rx, _properties) = can.split();
    let can_tx = make_multiplexable(can_tx);

    // Spawn activity LED
    spawner.spawn(blink::blink(led_yellow).expect("Failed to spawn blink task"));

    // Spawn power rail indicator LED task
    spawner.spawn(
        power_indicators::power_indicators(bat_p, ext_p, led_bat_p, led_ext_p)
            .expect("Failed to spawn power rail indicator task"),
    );

    // Spawn the buzzer alert task
    spawner.spawn(buzzer::buzzer_task(pwm).expect("Failed to spawn buzzer task"));

    // Spawn the two sensor readout tasks
    spawner.spawn(
        sensor_readout::sensor_readout_5v_task(ina_rail_5v)
            .expect("Failed to spawn 5V sensor task"),
    );

    spawner.spawn(
        sensor_readout::sensor_readout_24v_task(ina_rail_24v)
            .expect("Failed to spawn 24V sensor task"),
    );

    OUTPUTS
        .build_info
        .sender()
        .send(crate::build_info::BUILD_INFO.get().clone());
    spawner.spawn(can_board_status_task(can_tx).expect("Failed to spawn board status task"));
    OUTPUTS
        .start_broadcasting(spawner, can_tx)
        .expect("Failed to start CAN broadcasters");

    loop {
        match can_rx.recv().await {
            Ok(ReceivedMessage::ResetSpecific(board)) if board == THIS_BOARD_ID => {
                reset_now();
            }
            Ok(ReceivedMessage::ResetAll(_)) => {
                reset_now();
            }
            Ok(ReceivedMessage::UTCTimeUpdate(InstantUs(micros))) => {
                init_utc_clock(micros, Instant::now());
            }
            Ok(_) => {}
            Err(RxError::Bus(_)) => {
                Timer::after_millis(10).await;
            }
            Err(_) => {}
        }
    }
}

/// Reset the MCU immediately.
pub fn reset_now() {
    SCB::sys_reset();
}
