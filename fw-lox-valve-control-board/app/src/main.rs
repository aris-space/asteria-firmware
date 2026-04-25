#![no_std]
#![no_main]

mod blinky;
mod modbus_server;
mod valve;

use crate::blinky::status_blinky;
use core::future::pending;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::pac;
use embassy_stm32::usart::{self, DataBits, Parity, StopBits, Uart};
use embassy_stm32::{Config, bind_interrupts, dma, exti, peripherals};
use embedded_utils::fmt::*;

use crate::modbus_server::{
    DeviceStatus, clear_status, lox_valve_motor_controller_task,
    lox_valve_motor_position_updater_task, modbus_server_task, set_status,
};
use crate::valve::init_lox_valve_motor;

mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[cfg(not(feature = "defmt"))]
pub struct Db2F<'a, T: core::fmt::Debug + ?Sized>(pub &'a T);

#[cfg(feature = "defmt")]
pub use defmt::Debug2Format as Db2F;
use embassy_stm32::exti::ExtiInput;

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    USART2 => usart::InterruptHandler<peripherals::USART2>;

    // DMA channel interrupts
    DMA1_CHANNEL4 => dma::InterruptHandler<peripherals::DMA1_CH4>;
    DMA1_CHANNEL5 => dma::InterruptHandler<peripherals::DMA1_CH5>;
    DMA1_CHANNEL6 => dma::InterruptHandler<peripherals::DMA1_CH6>;
    DMA1_CHANNEL7 => dma::InterruptHandler<peripherals::DMA1_CH7>;

    // EXTI interrupt
    EXTI0 => exti::InterruptHandler<embassy_stm32::interrupt::typelevel::EXTI0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_stm32::init(Config::default());

    set_status(DeviceStatus::INITIALIZING);

    let x = pac::RCC.csr().read();
    debug!("{:?}", Db2F(&x));
    pac::RCC.csr().modify(|w| w.set_rmvf(true));

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

    let green = Output::new(p.PB8, Level::Low, Speed::Low);
    let red = Output::new(p.PB7, Level::Low, Speed::Low);

    // Endstop is active, when valve is fully closed.
    let endstop = ExtiInput::new(p.PA0, p.EXTI0, Pull::Up, Irqs);

    // --- Modbus UART Setup ---
    let mut cfg = usart::Config::default();
    cfg.baudrate = 115_200;
    cfg.parity = Parity::ParityNone;
    cfg.stop_bits = StopBits::STOP1;
    cfg.data_bits = DataBits::DataBits8;
    cfg.invert_rx = true;
    cfg.invert_tx = true;

    let modbus_usart = Uart::new_with_de(
        p.USART1, p.PA10, p.PA9, p.PA12, // DE (Driver Enable for RS485)
        p.DMA1_CH5, p.DMA1_CH4, Irqs, cfg,
    )
    .unwrap();

    let mut motor_cfg = usart::Config::default();
    motor_cfg.baudrate = 115_200;
    let motor_usart = Uart::new(
        p.USART2, p.PA3, p.PA2, p.DMA1_CH6, p.DMA1_CH7, Irqs, motor_cfg,
    )
    .unwrap();

    spawner.spawn(status_blinky(green, red).expect("Failed to spawn status blinky task"));
    spawner.spawn(modbus_server_task(modbus_usart).expect("Failed to spawn Modbus server task"));

    match init_lox_valve_motor(motor_usart, endstop).await {
        Ok(()) => {
            info!("System initialized successfully.");
            clear_status(DeviceStatus::INITIALIZING);
            set_status(DeviceStatus::READY);

            spawner.spawn(
                lox_valve_motor_position_updater_task()
                    .expect("Failed to spawn LOX valve motor position updater task"),
            );
            spawner.spawn(
                lox_valve_motor_controller_task()
                    .expect("Failed to spawn LOX valve motor controller task"),
            );
        }
        Err(e) => {
            error!("Fatal: Failed to initialize LOX valve motor: {:?}", e);
            clear_status(DeviceStatus::INITIALIZING);
        }
    }

    #[allow(unreachable_code)]
    loop {
        pending::<()>().await;
    }
}
