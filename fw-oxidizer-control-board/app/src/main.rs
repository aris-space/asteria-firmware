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
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, OutputType, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{bind_interrupts, can, dma, exti, i2c, peripherals};
use embassy_time::Timer;
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

use crate::actuators::dpr::pid_controller;
use crate::actuators::valves::valve_task;
use crate::buzzer::buzzer_task;
use crate::can_impl::{ReceivedMessage, board_status_update_task, can_rx_task};
use crate::globals::STATE;
use crate::sensors::OSS_TNK_T;
use crate::sensors::keller_analog_p::{OxidizerPressureHandles, oxidizer_pressure_acquisition};
use crate::sensors::thermocouples::thermocouple_task;
use ads1120_thermocouples::thermocouple_conversions::ThermocoupleType;
use ads1120_thermocouples::{ADSThermocouples, PGAGain};
use can_utils::broadcast::Broadcast as _;
use can_utils::setup::{make_multiplexable, setup_can};
use data_core::can::hal::CanDecode as _;
use max31889_thermistor::MAX31889;
#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;
use trafag_pressure::{config_vref_buf, set_adc_configs};

bind_interrupts!(struct Irqs {
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;

    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;

    // DMA channel interrupts
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    DMA1_CHANNEL3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
    DMA2_CHANNEL3 => dma::InterruptHandler<peripherals::DMA2_CH3>;

    // EXTI interrupt
    EXTI4 => exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI4>;
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
    set_adc_configs(&mut config);
    let p = embassy_stm32::init(config);

    // LEDs
    let green = Output::new(p.PB0, Level::High, Speed::Low);
    let yellow = Output::new(p.PB1, Level::High, Speed::Low);
    let red = Output::new(p.PB2, Level::High, Speed::Low);

    // Solenoids
    let oxidizer_vent_valve = Output::new(p.PB5, Level::Low, Speed::Medium);
    let oxidizer_dpr_valve = Output::new(p.PB6, Level::Low, Speed::VeryHigh);

    let ext_irq = ExtiInput::new(p.PC4, p.EXTI4, Pull::Up, Irqs);
    let tc = ADSThermocouples::new(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA1_CH1,
        p.DMA1_CH2,
        Irqs,
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
        p.DMA1_CH6,
        p.DMA1_CH7,
        Irqs,
        Default::default(),
    );

    let max = MAX31889::new(i2c, 0x50);

    let buzzer_pwm_pin = PwmPin::new(p.PB10, OutputType::PushPull);
    let buzzer_pwm = SimplePwm::new(
        p.TIM2,
        None,
        None,
        Some(buzzer_pwm_pin),
        None,
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

    // Can Bus
    let can = setup_can(p.FDCAN1, p.PB8, p.PB9, Irqs, ReceivedMessage::SUPPORTED_IDS);
    let (tx, rx, _) = can.split();
    let tx = make_multiplexable(tx).await;
    STATE
        .start_broadcasting(spawner, tx)
        .expect("failed to start CAN broadcasting");

    spawner.spawn(oxidizer_pressure_acquisition(pressure_handles).unwrap());

    spawner.spawn(thermocouple_task(tc, max, &OSS_TNK_T, &ThermocoupleType::K).unwrap());

    spawner.spawn(pid_controller(oxidizer_dpr_valve).expect("dpr task failed"));

    spawner.spawn(valve_task(oxidizer_vent_valve).expect("valve task failed"));

    spawner.spawn(can_rx_task(rx).unwrap());
    spawner.spawn(board_status_update_task().expect("board status task failed"));

    spawner.spawn(activity_blinky(green, yellow, red).unwrap());

    spawner.spawn(buzzer_task(buzzer_pwm).unwrap());

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
