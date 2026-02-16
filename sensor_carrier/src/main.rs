#![no_std]
#![no_main]
mod build_info;
mod can_impl;
mod drivers;
mod filters;
mod sensors;
mod util;

use crate::drivers::environmental::{ENVIRONMENTAL_DRIVER_PUBSUB, ENVIRONMENTAL_DRIVER_WATCH};
use crate::drivers::inertial::{ORIENTATION_PUBSUB, ORIENTATION_WATCH};
use crate::drivers::magnetic_field::{MAGNETIC_FIELD_PUBSUB, MAGNETIC_FIELD_WATCH};
use crate::drivers::{environmental, inertial, magnetic_field};
use crate::sensors::barometer::barometer_task;
use crate::sensors::dht::dht_task;
use crate::sensors::gnss::gnss_task;
use crate::sensors::imu::imu_task;
use crate::sensors::magnetometer::magnetometer_task;
use crate::sensors::{SHARED_BUS1, SHARED_BUS2, SensorId};
use core::future::pending;
use drivers::pressure;
use drivers::pressure::{PRESSURE_DRIVER_PUBSUB, PRESSURE_DRIVER_WATCH};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::peripherals::FDCAN3;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::{khz, mhz};
use embassy_stm32::usart::Uart;
use embassy_stm32::{bind_interrupts, can, i2c, peripherals, spi, usart};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use embedded_utils::fmt::*;
use lsm6dso32::spi::Lsm6Dso32SpiInterface;
use ms5607::Ms5607;
use sht4x::Sht4xAsync;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../shared/stm32h723_clocks.rs"
    ));
}

use clocks::clocks_config;

#[cfg(not(feature = "defmt"))]
pub struct Debug2Format<'a, T: core::fmt::Debug + ?Sized>(pub &'a T);

#[cfg(feature = "defmt")]
pub use defmt::Debug2Format;

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

// bind interrupts for UART/I2C/CAN (updated to I2C4/I2C5 and FDCAN3)
bind_interrupts!(struct Irqs {
    // GPS1 data on UART8 (PE0 RX, PE1 TX)
    UART8 => usart::InterruptHandler<peripherals::UART8>;

    // GPS2 data on UART7 (PE7 RX, PE8 TX)
    UART7 => usart::InterruptHandler<peripherals::UART7>;

    // I2C5 (bus 2)
    I2C5_EV => i2c::EventInterruptHandler<peripherals::I2C5>;
    I2C5_ER => i2c::ErrorInterruptHandler<peripherals::I2C5>;

    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
    // I2C4 (bus 1)
    //I2C4_EV => i2c::EventInterruptHandler<peripherals::I2C4>;
    //I2C4_ER => i2c::ErrorInterruptHandler<peripherals::I2C4>;


    // CAN on FDCAN3 (PF6 RX, PF7 TX)
    FDCAN3_IT0 => can::IT0InterruptHandler<FDCAN3>;
    FDCAN3_IT1 => can::IT1InterruptHandler<FDCAN3>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let config = clocks_config();
    let p = embassy_stm32::init(config);

    use build_info::built as built_info;
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

    // Buzzer PWM pin (TIM4_CH4) is on PD15 (not PB10)
    let _buzzer = Output::new(p.PD15, Level::Low, Speed::VeryHigh);

    // === UART config (GPS) ===
    let mut uart_config = usart::Config::default();
    uart_config.data_bits = usart::DataBits::DataBits8;
    uart_config.parity = usart::Parity::ParityNone;
    uart_config.stop_bits = usart::StopBits::STOP1;
    uart_config.baudrate = 921_600;

    // GPS1 data: UART8 (RX=PE0, TX=PE1)
    let gps1_data = Uart::new(
        p.UART8,
        p.PE0, // RX
        p.PE1, // TX
        Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        uart_config,
    )
    .expect("Failed to create GPS1 data UART");
    let (_gps1_tx, gps1_rx) = gps1_data.split();

    // GPS2 data: UART7 (RX=PE7, TX=PE8)
    let gps2_data = Uart::new(
        p.UART7,
        p.PE7, // RX
        p.PE8, // TX
        Irqs,
        p.DMA1_CH2,
        p.DMA1_CH3,
        uart_config,
    )
    .expect("Failed to initialize GPS2 data UART");
    let (_gps2_tx, gps2_rx) = gps2_data.split();

    // === IMU SPI buses/pins ===
    // IMU1 on SPI1: SCK=PG11, MOSI=PD7, MISO=PG9, CS=PG10, INT1=PE4 (EXTI4)
    let mut imu_spi_config: spi::Config = spi::Config::default();
    imu_spi_config.frequency = mhz(10);

    let imu1_spi = Spi::new(
        p.SPI1,
        p.PG11, // SCK
        p.PD7,  // MOSI
        p.PG9,  // MISO
        p.DMA1_CH4,
        p.DMA1_CH5,
        imu_spi_config,
    );
    let imu1_cs = Output::new(p.PG10, Level::High, Speed::VeryHigh);
    // INT must be left floating or pulled low during power-up.
    let imu1_int1 = ExtiInput::new(p.PE4, p.EXTI4, Pull::None);

    let imu1_spi = embedded_hal_bus::spi::ExclusiveDevice::new(imu1_spi, imu1_cs, Delay).expect(
        "Error while creating exclusive device. CS Pin set failed, which should never fail.",
    );

    // IMU2 on SPI4: SCK=PE12, MOSI=PE14, MISO=PE13, CS=PE11, INT1=PE15 (EXTI15)
    let imu2_spi_hw = Spi::new(
        p.SPI4,
        p.PE12, // SCK
        p.PE14, // MOSI
        p.PE13, // MISO
        p.DMA1_CH6,
        p.DMA1_CH7,
        imu_spi_config,
    );
    let imu2_cs = Output::new(p.PE11, Level::High, Speed::VeryHigh);
    let imu2_int1 = ExtiInput::new(p.PE15, p.EXTI15, Pull::None);

    let imu2_spi = embedded_hal_bus::spi::ExclusiveDevice::new(imu2_spi_hw, imu2_cs, Delay).expect(
        "Error while creating exclusive device. CS Pin set failed, which should never fail.",
    );

    // NOTE: Removed SPI2 flash stub that used PB12..PB15 to avoid conflicts with UART5 pins.

    // LEDs: LED3=PA8, LED2=PA9, LED1=PA10
    let _led_green = Output::new(p.PA8, Level::Low, Speed::Low);
    let led_yellow = Output::new(p.PA9, Level::Low, Speed::Low);
    let _led_red = Output::new(p.PA10, Level::Low, Speed::Low);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // I2C buses: use I2C4 (PF14=SCL, PF15=SDA) and I2C5 (PF1=SCL, PF0=SDA)
    let mut i2c_bus_config: i2c::Config = i2c::Config::default();
    i2c_bus_config.timeout = Duration::from_millis(50);
    let i2c_bus_freq = khz(100);

    // Bus1 -> I2C5
    let bus1 = i2c::I2c::new(
        p.I2C5,
        p.PF1, // SCL
        p.PF0, // SDA
        Irqs,
        p.DMA2_CH2,
        p.DMA2_CH7,
        i2c_bus_freq,
        i2c_bus_config,
    );

    // Bus2 -> I2C4, but actually I2C2 because we bridged it!
    let bus2 = i2c::I2c::new(
        p.I2C2,
        p.PB10,
        p.PB11,
        Irqs,
        p.DMA2_CH0,
        p.DMA2_CH1,
        i2c_bus_freq,
        i2c_bus_config,
    );

    // === Initialize Shared I2C Buses ===
    SHARED_BUS1
        .init(Mutex::<ThreadModeRawMutex, _>::new(bus1))
        .ok()
        .expect("Could not initialize shared bus.");

    SHARED_BUS2
        .init(Mutex::<ThreadModeRawMutex, _>::new(bus2))
        .ok()
        .expect("Could not initialize shared bus.");

    let shared_bus1 = SHARED_BUS1.get().await;
    let shared_bus2 = SHARED_BUS2.get().await;

    // === External Interrupts (compass DRDY/INT) ===
    // Magnetometer 1 -> PF2 (EXTI2)
    let _compass_interrupt_bus1 = ExtiInput::new(p.PF2, p.EXTI2, Pull::None);
    // Magnetometer 2 -> PG0 (EXTI0)
    let _compass_interrupt_bus2 = ExtiInput::new(p.PG0, p.EXTI0, Pull::None);

    // === Drivers Initialization ===

    // --- Pressure ---
    let pressure_driver = pressure::PressureDriver::new(
        PRESSURE_DRIVER_PUBSUB.immediate_publisher(),
        PRESSURE_DRIVER_WATCH.sender(),
    );
    pressure::PRESSURE_DRIVER
        .init(pressure_driver)
        .ok()
        .expect("Could not initialize pressure driver.");
    let pressure_driver = pressure::PRESSURE_DRIVER.get().await;

    // --- Environmental ---
    let environmental_driver = environmental::EnvironmentalDriver::new(
        ENVIRONMENTAL_DRIVER_PUBSUB.immediate_publisher(),
        ENVIRONMENTAL_DRIVER_WATCH.sender(),
    );
    environmental::ENVIRONMENTAL_DRIVER
        .init(environmental_driver)
        .ok()
        .expect("Could not initialize environmental driver.");
    let environmental_driver = environmental::ENVIRONMENTAL_DRIVER.get().await;

    // --- Magnetic Field ---
    let magfield_driver = magnetic_field::MagneticFieldDriver::new(
        MAGNETIC_FIELD_PUBSUB.immediate_publisher(),
        MAGNETIC_FIELD_WATCH.sender(),
    );
    magnetic_field::MAGNETIC_FIELD_DRIVER
        .init(magfield_driver)
        .ok()
        .expect("Could not initialize magnetic field driver.");
    let magfield_driver = magnetic_field::MAGNETIC_FIELD_DRIVER.get().await;

    // --- Orientation ---
    let orientation_driver = inertial::InertialDriver::new(
        ORIENTATION_PUBSUB.immediate_publisher(),
        ORIENTATION_WATCH.sender(),
        inertial::INERTIAL_PUBSUB.immediate_publisher(),
        inertial::INERTIAL_WATCH.sender(),
        MAGNETIC_FIELD_WATCH
            .receiver()
            .expect("Failed to create magnetometer watch."),
    );
    inertial::INERTIAL_DRIVER
        .init(orientation_driver)
        .ok()
        .expect("Could not initialize orientation driver.");
    let orientation_driver = inertial::INERTIAL_DRIVER.get().await;

    // --- Position ---
    let position_driver = drivers::position_velocity::PositionVelocityTimeDriver::new(
        drivers::position_velocity::POSITION_PUBSUB.immediate_publisher(),
        drivers::position_velocity::POSITION_WATCH.sender(),
        drivers::position_velocity::VELOCITY_PUBSUB.immediate_publisher(),
        drivers::position_velocity::VELOCITY_WATCH.sender(),
    );
    drivers::position_velocity::POSITION_VELOCITY_TIME_DRIVER
        .init(position_driver)
        .ok()
        .expect("Could not initialize position driver.");
    let position_driver = drivers::position_velocity::POSITION_VELOCITY_TIME_DRIVER
        .get()
        .await;

    // === Sensor Buses ===
    let ms5607_bus1 = Ms5607::new(I2cDevice::new(shared_bus1), false);
    let ms5607_bus2 = Ms5607::new(I2cDevice::new(shared_bus2), false);

    let sht_bus1: Sht4xAsync<_, Delay> = Sht4xAsync::new(I2cDevice::new(shared_bus1));
    let sht_bus2: Sht4xAsync<_, Delay> = Sht4xAsync::new(I2cDevice::new(shared_bus2));

    let compass_bus1 = I2cDevice::new(shared_bus1);
    let compass_bus2 = I2cDevice::new(shared_bus2);

    // === Spawn Sensor Tasks ===

    // --- IMU ---
    spawner
        .spawn(imu_task(
            Lsm6Dso32SpiInterface { spi: imu1_spi },
            orientation_driver,
            SensorId::Imu1,
            imu1_int1,
        ))
        .expect("Error spawning IMU 1 task.");

    spawner
        .spawn(imu_task(
            Lsm6Dso32SpiInterface { spi: imu2_spi },
            orientation_driver,
            SensorId::Imu2,
            imu2_int1,
        ))
        .expect("Error spawning IMU 2 task.");

    // --- Magnetometer ---
    spawner
        .spawn(magnetometer_task(
            compass_bus1,
            magfield_driver,
            SensorId::MagnetometerBus1,
        ))
        .expect("Error spawning magnetometer 1 task.");

    spawner
        .spawn(magnetometer_task(
            compass_bus2,
            magfield_driver,
            SensorId::MagnetometerBus2,
        ))
        .expect("Error spawning magnetometer 2 task.");

    // --- Barometer ---
    spawner
        .spawn(barometer_task(
            ms5607_bus1,
            pressure_driver,
            environmental_driver,
            SensorId::BarometerBus1,
        ))
        .expect("Error spawning barometer 1 task.");

    spawner
        .spawn(barometer_task(
            ms5607_bus2,
            pressure_driver,
            environmental_driver,
            SensorId::BarometerBus2,
        ))
        .expect("Error spawning barometer 2 task.");

    // --- DHT (Environmental) ---
    spawner
        .spawn(dht_task(sht_bus1, environmental_driver, SensorId::DhtBus1))
        .expect("Error spawning DHT 1 task.");

    spawner
        .spawn(dht_task(sht_bus2, environmental_driver, SensorId::DhtBus2))
        .expect("Error spawning DHT 2 task.");

    // --- GPS ---
    spawner
        .spawn(gnss_task(gps1_rx, position_driver, SensorId::Gps1))
        .expect("Error spawning GPS1 task.");

    spawner
        .spawn(gnss_task(gps2_rx, position_driver, SensorId::Gps2))
        .expect("Error spawning GPS2 task.");

    // === Other Tasks ===
    spawner
        .spawn(blink(led_yellow))
        .expect("Error spawning blinking task.");

    // setup CAN on FDCAN3 PF6/PF7 and start the loops
    let can = can_impl::setup_can(p.FDCAN3, p.PF6, p.PF7, Irqs); // TX=PF7, RX=PF6
    let (tx, rx, _options) = can.split();

    spawner
        .spawn(can_impl::can_rx_task(rx))
        .expect("Failed to spawn CAN RX task.");

    can_impl::spawn_can_tx_tasks(tx, spawner).await;

    #[allow(unreachable_code)]
    loop {
        pending::<()>().await;
    }
}

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(300).await;
        led.set_low();
        Timer::after_millis(700).await;
    }
}
