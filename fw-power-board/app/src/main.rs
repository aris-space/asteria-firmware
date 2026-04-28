// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

mod blink;
mod board;
mod build_info;
mod buzzer;
mod can;
mod can_impl;
mod sensor_readout;
mod unix_time;

use crate::can::spawn_can_tasks;
use board::{INA232_I2C_ADDR, Irqs};
use cortex_m::peripheral::SCB;
use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_time::Duration;
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

    // Configure LEDs (PB0, PB1, PB2) as outputs (low = off)
    let _led_green = Output::new(p.PB0, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PB1, Level::Low, Speed::Low);
    let _led_red = Output::new(p.PB2, Level::Low, Speed::Low);

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
    let can = can_impl::setup_can(p.FDCAN1, p.PB8, p.PB9, Irqs);
    let (can_tx, can_rx, _properties) = can.split();

    // Spawn activity LED
    spawner.spawn(blink::blink(led_yellow).expect("Failed to spawn blink task"));

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

    // Spawn the CAN tasks
    spawn_can_tasks(&spawner, can_rx, can_tx).await;

    // Prevent the main task from exiting
    #[allow(unreachable_code)]
    loop {
        core::future::pending::<()>().await;
    }
}

/// Reset the MCU immediately.
pub fn reset_now() {
    SCB::sys_reset();
}
