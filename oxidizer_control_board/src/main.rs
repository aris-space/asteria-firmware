#![no_std]
#![no_main]
mod actuators;
mod build_info;
mod buzzer;
mod can_impl;
mod drivers;
mod sensors;

use core::future::pending;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, OutputType, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::{FDCAN1, USART3};
use embassy_stm32::time::{Hertz, khz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{bind_interrupts, can, i2c, peripherals, usart};
use embassy_time::Timer;
use embedded_utils::fmt::*;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../shared/stm32g473_clocks.rs"
    ));
}

use clocks::clocks_config;

mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use crate::sensors::keller_p::keller_acquisition;

use crate::actuators::dpr::pid_controller;
use crate::actuators::valves::valve_task;
use crate::buzzer::buzzer_task;
use crate::can_impl::{can_rx_task, setup_can, spawn_can_tx_task};
use crate::sensors::FSS_TNK_T;
use crate::sensors::thermocouples::thermocouple_task;
use ads1120_thermocouples::thermocouple_conversions::ThermocoupleType;
use ads1120_thermocouples::{ADSThermocouples, PGAGain};
use keller_pressure::KellerSensRS485;
use max31889_thermistor::MAX31889;
#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

bind_interrupts!(struct Irqs {
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;

    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;

    USART3 => usart::InterruptHandler<USART3>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
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

    let config = clocks_config();
    let p = embassy_stm32::init(config);

    // LEDs
    let green = Output::new(p.PB0, Level::High, Speed::Low);
    let yellow = Output::new(p.PB1, Level::High, Speed::Low);
    let red = Output::new(p.PB2, Level::High, Speed::Low);

    // Solenoids
    let oss_vnt = Output::new(p.PB5, Level::Low, Speed::Medium);

    let dpr_pin = Output::new(p.PB6, Level::Low, Speed::VeryHigh);

    // RS485
    let keller_handle = KellerSensRS485::new(
        p.USART3, p.PC11, p.PC10, p.DMA2_CH1, p.DMA1_CH1, Irqs, p.PC12,
    )
    .await
    .unwrap();

    let ext_irq = ExtiInput::new(p.PC4, p.EXTI4, Pull::Up);
    let tc = ADSThermocouples::new(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA1_CH3,
        p.DMA1_CH2,
        p.PA4,
        ext_irq,
        PGAGain::Gain32,
    )
    .await
    .unwrap();

    // I2C
    let i2c = I2c::new(
        p.I2C3,
        p.PC8,
        p.PC9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        khz(100),
        Default::default(),
    );

    let max = MAX31889::new(i2c, 0x50);

    let buzzer_pwm_pin = PwmPin::new_ch3(p.PB10, OutputType::PushPull);
    let buzzer_pwm = SimplePwm::new(
        p.TIM2,
        None,
        None,
        Some(buzzer_pwm_pin),
        None,
        Hertz(440),
        Default::default(),
    );

    // Can Bus
    let can = setup_can(p.FDCAN1, p.PB8, p.PB9, Irqs);
    let (tx, rx, _) = can.split();

    spawner.spawn(keller_acquisition(keller_handle)).unwrap();

    spawner
        .spawn(thermocouple_task(tc, max, &FSS_TNK_T, &ThermocoupleType::K))
        .unwrap();

    spawner
        .spawn(pid_controller(dpr_pin))
        .expect("dpr task failed");

    spawner
        .spawn(valve_task(oss_vnt))
        .expect("valve task failed");

    spawner.spawn(can_rx_task(rx)).unwrap();

    spawner.spawn(activity_blinky(green, yellow, red)).unwrap();

    spawn_can_tx_task(tx, spawner).await;

    spawner.spawn(buzzer_task(buzzer_pwm)).unwrap();

    #[allow(unreachable_code)]
    loop {
        pending::<()>().await;
    }
}

#[embassy_executor::task]
async fn activity_blinky(
    mut led1: Output<'static>,
    mut led2: Output<'static>,
    mut led3: Output<'static>,
) {
    loop {
        led1.set_high();
        Timer::after_millis(100).await;
        led1.set_low();
        led2.set_high();
        Timer::after_millis(100).await;
        led2.set_low();
        led3.set_high();
        Timer::after_millis(100).await;
        led3.set_low();
        Timer::after_millis(700).await;
    }
}
