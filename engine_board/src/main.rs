#![no_std]
#![no_main]

mod build_info;
mod buzzer;
mod can_impl;
mod controls;
mod drivers;
mod dummy_thrust_curve;
pub(crate) mod k23_temperature_control;
mod sensors;
mod valves;

use core::future::pending;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Speed};
use embassy_stm32::{bind_interrupts, can, i2c, peripherals, usart};
use embassy_time::{Duration, Ticker};
use embedded_utils::clocks_config;
use embedded_utils::fmt::*;

mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use crate::buzzer::buzzer_task;
use crate::can_impl::{can_rx_task, setup_can, spawn_can_tx_task};
use crate::controls::runner::initiate_runner_tasks;
use crate::k23_temperature_control::k23_temperature_control;
use crate::sensors::keller_p::keller_acquisition;
use crate::sensors::{OXD_RNL_T, OXD_TNK_T};
use crate::valves::{check_main_arming, valve_task};
use ads1120_thermocouples::{ADSThermocouples, PGAGain};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::i2c::I2c;
use embassy_stm32::peripherals::{FDCAN1, USART3};
use embassy_stm32::time::{khz, Hertz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use keller_pressure::KellerSensRS485;
use max31889_thermistor::MAX31889;
#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;
use sensors::thermocouples::thermocouple_task;
use trafag_pressure::{config_vref_buf, set_adc_configs};

bind_interrupts!(struct Irqs {
    I2C3_EV => i2c::EventInterruptHandler<peripherals::I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<peripherals::I2C3>;

    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;

    USART3 => usart::InterruptHandler<USART3>;
});

pub static CRICITAL_ERROR_INDICATOR: OnceLock<Mutex<NoopRawMutex, Output<'static>>> =
    OnceLock::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    debug!("pkg_name: {}, git_commit_hash_short: {}, git_dirty: {}, profile: {}, features: {}, rustc: {}, target: {}",
        built_info::PKG_NAME, built_info::GIT_COMMIT_HASH_SHORT, built_info::GIT_DIRTY, built_info::PROFILE, built_info::FEATURES, built_info::RUSTC, built_info::TARGET);

    let mut config = clocks_config();
    set_adc_configs(&mut config);

    let p = embassy_stm32::init(config);

    // LEDs
    let green = Output::new(p.PB0, Level::High, Speed::Low);
    let yellow = Output::new(p.PB1, Level::High, Speed::Low);

    CRICITAL_ERROR_INDICATOR
        .init(Mutex::new(Output::new(p.PB2, Level::Low, Speed::Low)))
        .ok()
        .unwrap();

    config_vref_buf();

    // Analog Pressure Initialization
    /*
    let eng_cc_p_handle = ADCPressure::new(
        p.ADC3,
        p.DMA1_CH3,
        TrafagPSens {
            pin: p.PB0.degrade_adc(),
            si_range: ENG_CC_P_RANGE,
        },
    )
    .await;

    let fss_inj_p_handle = ADCPressure::new(
        p.ADC1,
        p.DMA1_CH4,
        TrafagPSens {
            pin: p.PB1.degrade_adc(),
            si_range: FUE_INJ_P_RANGE,
        },
    )
    .await;

    let oss_inj_p_handle = ADCPressure::new(
        p.ADC2,
        p.DMA2_CH3,
        TrafagPSens {
            pin: p.PB2.degrade_adc(),
            si_range: OXD_INJ_P_RANGE,
        },
    )
    .await;

    let eng_p_handles = EnginePressureHandles {
        eng_cc_p_handle,
        fss_inj_p_handle,
        oss_inj_p_handle,
    };
    */

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

    let oss_mnl_vlv = Output::new(p.PB4, Level::Low, Speed::VeryHigh);
    let fss_mnl_vlv = Output::new(p.PB5, Level::Low, Speed::VeryHigh);

    let heating_pad_switching = Output::new(p.PB6, Level::Low, Speed::Medium);

    let main_arming_pin = Input::new(p.PA0, Pull::Down);

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

    // RS485f
    let keller_handle = KellerSensRS485::new(
        p.USART3, p.PC11, p.PC10, p.DMA2_CH1, p.DMA1_CH1, Irqs, p.PC12,
    )
    .await
    .unwrap();

    // Can Bus
    let can = setup_can(p.FDCAN1, p.PB8, p.PB9, Irqs);
    let (tx, rx, _) = can.split();

    // Using digital pressures so this here not used
    /*spawner
    .spawn(engine_pressure_acquisition(eng_p_handles))
    .expect("Engine Pressure Acquisition Task Failed");*/

    spawner
        .spawn(keller_acquisition(keller_handle))
        .expect("could not spawn keller thread");

    spawner
        .spawn(thermocouple_task(tc, max, &OXD_RNL_T, &OXD_TNK_T))
        .expect("Thermocouple Task failed");

    spawner
        .spawn(valve_task(oss_mnl_vlv, fss_mnl_vlv))
        .expect("Valve Task failed");

    spawner
        .spawn(check_main_arming(main_arming_pin))
        .expect("Check main arming Task failed");

    spawner
        .spawn(can_rx_task(rx, yellow))
        .expect("Can't spawn CAN RX task");

    spawn_can_tx_task(tx, spawner).await;

    spawner
        .spawn(activity_blinky(green))
        .expect("blinky executor");

    initiate_runner_tasks(spawner)
        .await
        .expect("InitiateRunnerTasks failed");

    spawner
        .spawn(k23_temperature_control(heating_pad_switching))
        .expect("K23 temperature control failed");

    spawner.spawn(buzzer_task(buzzer_pwm)).unwrap();

    info!("ALL TASKS SPAWNED");

    loop {
        pending::<()>().await;
    }
}

#[embassy_executor::task]
async fn activity_blinky(mut green: Output<'static>) {
    let mut ticker = Ticker::every(Duration::from_millis(500));

    loop {
        green.toggle();
        ticker.next().await;
    }
}

pub async fn indicate_critical_error() {
    CRICITAL_ERROR_INDICATOR.get().await.lock().await.set_high();
}
