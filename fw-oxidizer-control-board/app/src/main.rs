#![no_std]
#![no_main]
mod actuators;
mod build_info;
mod buzzer;
mod can_impl;
mod drivers;
mod globals;
mod sensors;

use core::future::pending;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, OutputType, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::rcc::mux;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{bind_interrupts, can, dma, i2c, peripherals};
use embassy_time::{Duration, Timer};
use embedded_utils::fmt::*;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32g473_clocks.rs"
    ));
}

use clocks::clocks_config;

mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use crate::actuators::valves::valve_task;
use crate::buzzer::buzzer_task;
use crate::can_impl::{ReceivedMessage, board_status_update_task, can_rx_task};
use crate::drivers::solenoid_detection::solenoid_detection_task;
use crate::globals::STATE;
use crate::sensors::keller_analog_p::{OxidizerPressureHandles, oxidizer_pressure_acquisition};
use crate::sensors::solenoid_current::solenoid_current_task;
use analog_pressure::config_vref_buf;
use can_utils::broadcast::Broadcast as _;
use can_utils::setup::{make_multiplexable, setup_can};
use data_core::can::hal::CanDecode as _;
use dpr::pid_controller;
#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

bind_interrupts!(struct Irqs {
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;

    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;

    // DMA channel interrupts
    DMA1_CHANNEL3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
    DMA2_CHANNEL3 => dma::InterruptHandler<peripherals::DMA2_CH3>;
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

    let mut config = clocks_config();
    config.rcc.mux.adc12sel = mux::Adcsel::SYS;
    config.rcc.mux.adc345sel = mux::Adcsel::SYS;
    let p = embassy_stm32::init(config);

    // LEDs
    let _green = Output::new(p.PC15, Level::High, Speed::Low);
    let _yellow = Output::new(p.PC14, Level::High, Speed::Low);
    let red = Output::new(p.PC13, Level::High, Speed::Low);

    // Solenoids
    let oxidizer_vent_valve = Output::new(p.PA10, Level::Low, Speed::Medium);
    let oxidizer_dpr_valve = Output::new(p.PB11, Level::Low, Speed::VeryHigh);

    // Solenoid Standby Control
    let mut sol1_stby = Output::new(p.PB15, Level::High, Speed::Low);
    let mut sol2_stby = Output::new(p.PB1, Level::High, Speed::Low);
    sol1_stby.set_high();
    sol2_stby.set_high();

    let buzzer_pwm_pin = PwmPin::new(p.PB7, OutputType::PushPull);
    let buzzer_pwm = SimplePwm::new(
        p.TIM3,
        None,
        None,
        None,
        Some(buzzer_pwm_pin),
        Hertz(440),
        Default::default(),
    );

    config_vref_buf();

    // Trafag analog pressure sensors
    let pressure_handles = OxidizerPressureHandles {
        oxidizer_tank_pressure_1_adc: p.ADC1,
        oxidizer_tank_pressure_1_dma: p.DMA1_CH4,
        oxidizer_tank_pressure_1_pin: p.PC0,
        oxidizer_tank_pressure_2_adc: p.ADC2,
        oxidizer_tank_pressure_2_dma: p.DMA2_CH3,
        oxidizer_tank_pressure_2_pin: p.PC1,
        oxidizer_tank_differential_pressure_adc: p.ADC3,
        oxidizer_tank_differential_pressure_dma: p.DMA1_CH3,
        oxidizer_tank_differential_pressure_pin: p.PB13,
    };

    let mut i2c_config = i2c::Config::default();
    i2c_config.timeout = Duration::from_millis(50);
    i2c_config.frequency = Hertz(100_000);

    // ADS1015 solenoid current monitor on I2C3: SCL = PC8, SDA = PC9.
    let solenoid_current_i2c = I2c::new(
        p.I2C3, p.PC8, p.PC9, p.DMA1_CH6, p.DMA1_CH7, Irqs, i2c_config,
    );

    // Can Bus
    let can = setup_can(p.FDCAN1, p.PB8, p.PB9, Irqs, ReceivedMessage::SUPPORTED_IDS);
    let (tx, rx, _) = can.split();
    let tx = make_multiplexable(tx);
    STATE
        .start_broadcasting(spawner, tx)
        .expect("failed to start CAN broadcasting");

    spawner.spawn(oxidizer_pressure_acquisition(pressure_handles).unwrap());

    spawner.spawn(
        pid_controller(
            oxidizer_dpr_valve,
            STATE.dpr_control_loop.receiver().unwrap(),
            STATE.dpr_pressure.receiver().unwrap(),
            STATE.dpr_gain.receiver().unwrap(),
            STATE.dpr_info.sender(),
        )
        .expect("failed to prepare pid_controller spawn token"),
    );

    spawner.spawn(valve_task(oxidizer_vent_valve).expect("valve task failed"));

    spawner.spawn(can_rx_task(rx).unwrap());
    spawner.spawn(board_status_update_task().expect("board status task failed"));

    spawner.spawn(build_status_blinky(red).expect("build status blinky task failed"));

    spawner.spawn(buzzer_task(buzzer_pwm).unwrap());

    spawner.spawn(
        solenoid_detection_task(p.PC2, p.PB6).expect("failed to prepare solenoid detection task"),
    );

    spawner.spawn(
        solenoid_current_task(solenoid_current_i2c)
            .expect("failed to prepare solenoid current task"),
    );

    #[allow(unreachable_code)]
    loop {
        pending::<()>().await;
    }
}

#[embassy_executor::task]
async fn build_status_blinky(mut red: Output<'static>) {
    let build_info = crate::build_info::BUILD_INFO.get();
    let warning_build =
        build_info.is_git_dirty || !build_info.is_release || build_info.debug_defmt_rtt;
    let (on_ms, off_ms) = if warning_build {
        (125, 125)
    } else {
        (200, 1800)
    };

    loop {
        red.set_low();
        Timer::after_millis(on_ms).await;
        red.set_high();
        Timer::after_millis(off_ms).await;
    }
}
