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

use crate::board::{LIVESTREAM_CAMERA, RECORDING_CAMERA};
use crate::can::spawn_can_tasks;
use board::{Irqs, LTC2945_I2C_ADDR};
use cortex_m::peripheral::SCB;
use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;
use ltc2945::Ltc2945;

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
pub use ::panic_reset as _;

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    // Initialize clocks and peripherals
    let config = embedded_utils::clocks_config();
    let p = embassy_stm32::init(config);

    // Configure LEDs (PB0, PB1, PB2) as outputs (low = off)
    let _led_green = Output::new(p.PB0, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PB1, Level::Low, Speed::Low);
    let _led_red = Output::new(p.PB2, Level::Low, Speed::Low);

    // Configure camera power-control pins (PA3 = REC, PA4 = LIVE)
    let rec_pin = Output::new(p.PA3, Level::Low, Speed::Low);
    RECORDING_CAMERA
        .init(Mutex::new(rec_pin))
        .ok()
        .expect("Failed to set recording camera pin");
    let live_pin = Output::new(p.PA4, Level::Low, Speed::Low);
    LIVESTREAM_CAMERA
        .init(Mutex::new(live_pin))
        .ok()
        .expect("Failed to set livestream camera pin");

    // Shared I2C configuration: 100 ms timeout, 400 kHz speed
    let mut shared_i2c_config = i2c::Config::default();
    shared_i2c_config.timeout = Duration::from_millis(5);

    // Initialize I2C3 (SDA = PC9, SCL = PC8) for the 5V sensor
    let i2c3 = I2c::new(
        p.I2C3,
        p.PC8, // SCL
        p.PC9, // SDA
        Irqs,
        p.DMA2_CH3,
        p.DMA2_CH4,
        Hertz(400_000),
        shared_i2c_config,
    );
    let ltc_rail_5v = Ltc2945::new_i2c(i2c3, LTC2945_I2C_ADDR);

    // Initialize I2C2 (SDA = PA8, SCL = PA9) for the 24V sensor
    let i2c2 = I2c::new(
        p.I2C2,
        p.PA9, // SCL
        p.PA8, // SDA
        Irqs,
        p.DMA2_CH1,
        p.DMA2_CH2,
        Hertz(400_000),
        shared_i2c_config,
    );
    let ltc_rail_24v = Ltc2945::new_i2c(i2c2, LTC2945_I2C_ADDR);

    // Initialize PWM (TIM2 CH3 on PB10) for buzzer (~3 kHz, 50% duty)
    let buzzer_pin = PwmPin::new_ch3(p.PB10, OutputType::PushPull);
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
    spawner
        .spawn(blink::blink(led_yellow))
        .expect("Failed to spawn blink task");

    // Spawn the buzzer alert task
    spawner
        .spawn(buzzer::buzzer_task(pwm))
        .expect("Failed to spawn buzzer task");

    // Spawn the two sensor readout tasks
    spawner
        .spawn(sensor_readout::sensor_readout_5v_task(ltc_rail_5v))
        .expect("Failed to spawn 5V sensor task");

    spawner
        .spawn(sensor_readout::sensor_readout_24v_task(ltc_rail_24v))
        .expect("Failed to spawn 24V sensor task");

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
