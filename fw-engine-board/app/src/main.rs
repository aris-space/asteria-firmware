#![no_std]
#![no_main]

mod build_info;
mod buzzer;
mod can_impl;
mod drivers;
mod globals;
pub(crate) mod k23_temperature_control;
mod sensors;
mod valves;

use core::future::pending;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Speed};
use embassy_stm32::{bind_interrupts, can, dma, exti, i2c, peripherals};
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

use crate::buzzer::buzzer_task;
use crate::can_impl::{ReceivedMessage, board_status_update_task, can_rx_task};
use crate::drivers::solenoid_detection::solenoid_detection_task;
use crate::globals::STATE;
use crate::k23_temperature_control::k23_temperature_control;
use crate::sensors::solenoid_current::solenoid_current_task;
use crate::sensors::{OXD_RNL_T, OXD_TNK_T};
use crate::valves::{check_main_arming, valve_task};
use ads1120_thermocouples::{ADSThermocouples, PGAGain};
use can_utils::broadcast::Broadcast as _;
use can_utils::setup::{make_multiplexable, setup_can};
use data_core::can::hal::CanDecode as _;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use max31889_thermistor::MAX31889;
#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;
use sensors::thermocouples::thermocouple_task;
use sensors::trafag_p::{EnginePressureHandles, engine_pressure_acquisition};
use trafag_pressure::config_vref_buf;

bind_interrupts!(struct Irqs {
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;
    I2C4_EV => i2c::EventInterruptHandler<peripherals::I2C4>;
    I2C4_ER => i2c::ErrorInterruptHandler<peripherals::I2C4>;

    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;

    // DMA channel interrupts
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
    DMA1_CHANNEL3 => dma::InterruptHandler<peripherals::DMA1_CH3>;
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => dma::InterruptHandler<peripherals::DMA1_CH7>;
    DMA2_CHANNEL1 => dma::InterruptHandler<peripherals::DMA2_CH1>;
    DMA2_CHANNEL2 => dma::InterruptHandler<peripherals::DMA2_CH2>;
    DMA2_CHANNEL3 => dma::InterruptHandler<peripherals::DMA2_CH3>;

    // EXTI interrupt
    EXTI4 => exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI4>;
});

pub static CRICITAL_ERROR_INDICATOR: OnceLock<Mutex<ThreadModeRawMutex, Output<'static>>> =
    OnceLock::new();

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
    let _green = Output::new(p.PC15, Level::High, Speed::Low);
    let red = Output::new(p.PC13, Level::High, Speed::Low);

    //Used the yellow LED as critical error indicator
    CRICITAL_ERROR_INDICATOR
        .init(Mutex::new(Output::new(p.PC14, Level::Low, Speed::Low)))
        .ok()
        .unwrap();

    config_vref_buf();

    // Analog Pressure Initialization
    let eng_p_handles = EnginePressureHandles {
        eng_cc_p_adc: p.ADC1,
        eng_cc_p_dma: p.DMA1_CH4,
        eng_cc_p_pin: p.PC0,
        eng_inj_p_adc: p.ADC2,
        eng_inj_p_dma: p.DMA2_CH3,
        eng_inj_p_pin: p.PC1,
        oss_inj_p_adc: p.ADC3,
        oss_inj_p_dma: p.DMA2_CH2,
        oss_inj_p_pin: p.PB13,
    };

    let ext_irq = ExtiInput::new(p.PC4, p.EXTI4, Pull::Up, Irqs);
    let tc = ADSThermocouples::new(
        p.SPI1,
        p.PA5,
        p.PA7,
        p.PA6,
        p.DMA1_CH3,
        p.DMA1_CH2,
        Irqs,
        p.PA4,
        ext_irq,
        PGAGain::Gain32,
    )
    .await
    .unwrap();

    let mut i2c_config = i2c::Config::default();
    i2c_config.timeout = Duration::from_millis(50);
    i2c_config.frequency = Hertz(100_000);

    // I2C4 MAX31889 cold-junction sensor: SCL = PC6, SDA = PC7.
    let max_i2c = I2c::new(
        p.I2C4, p.PC6, p.PC7, p.DMA1_CH1, p.DMA2_CH1, Irqs, i2c_config,
    );
    let max = MAX31889::new(max_i2c, 0x50);

    // ADS1015 solenoid current monitor on I2C3: SCL = PC8, SDA = PC9.
    let solenoid_current_i2c = I2c::new(
        p.I2C3,
        p.PC8,
        p.PC9,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Irqs,
        Default::default(),
    );

    //Solenoids
    let oss_ml_vlv = Output::new(p.PB11, Level::Low, Speed::VeryHigh);
    let fss_ml_vlv = Output::new(p.PA10, Level::Low, Speed::VeryHigh);

    // Solenoid Standby Control
    let mut sol1_stby = Output::new(p.PB15, Level::High, Speed::Low);
    let mut sol2_stby = Output::new(p.PB1, Level::High, Speed::Low);
    sol1_stby.set_high();
    sol2_stby.set_high();

    let heating_pad_switching = Output::new(p.PB2, Level::Low, Speed::Medium);

    let main_arming_pin = Input::new(p.PC12, Pull::Down);

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

    // Can Bus
    let can = setup_can(p.FDCAN1, p.PB8, p.PB9, Irqs, ReceivedMessage::SUPPORTED_IDS);
    let (tx, rx, _) = can.split();
    let tx = make_multiplexable(tx);
    STATE
        .start_broadcasting(spawner, tx)
        .expect("failed to start CAN broadcasting");

    spawner.spawn(engine_pressure_acquisition(eng_p_handles).expect("Engine Pressure Task failed"));

    spawner.spawn(
        thermocouple_task(tc, max, &OXD_RNL_T, &OXD_TNK_T).expect("Thermocouple Task failed"),
    );

    spawner.spawn(valve_task(oss_ml_vlv, fss_ml_vlv).expect("Valve Task failed"));

    spawner.spawn(check_main_arming(main_arming_pin).expect("Check main arming Task failed"));

    spawner.spawn(can_rx_task(rx).expect("Can't spawn CAN RX task"));
    spawner.spawn(board_status_update_task().expect("Board Status Task failed"));

    spawner.spawn(build_status_blinky(red).expect("blinky executor"));

    spawner.spawn(
        k23_temperature_control(heating_pad_switching).expect("K23 temperature control failed"),
    );

    spawner.spawn(buzzer_task(buzzer_pwm).unwrap());
    // TODO: Replace PC2/PC3/PA1 with the correct solenoid detection pins.
    spawner.spawn(
        solenoid_detection_task(p.PB6, p.PC2, p.PA1)
            .expect("failed to prepare solenoid detection task"),
    );
    spawner.spawn(solenoid_current_task(solenoid_current_i2c).unwrap());

    info!("ALL TASKS SPAWNED");

    loop {
        pending::<()>().await;
    }
}

#[embassy_executor::task]
async fn build_status_blinky(mut red: Output<'static>) {
    let build_info = crate::build_info::BUILD_INFO.get();
    let warning_build = build_info.is_git_dirty || !build_info.is_release;
    let (on_ms, off_ms) = if warning_build {
        (125, 125)
    } else {
        (900, 100)
    };

    loop {
        red.set_low();
        Timer::after_millis(on_ms).await;
        red.set_high();
        Timer::after_millis(off_ms).await;
    }
}

pub async fn indicate_critical_error() {
    CRICITAL_ERROR_INDICATOR.get().await.lock().await.set_high();
}
