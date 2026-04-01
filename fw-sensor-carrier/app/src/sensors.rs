#![allow(dead_code)]
use core::sync::atomic::Ordering;
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::usart::Uart;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;

pub mod barometer;
pub mod dht;
pub mod gnss;
pub mod imu;
pub mod magnetometer;

pub static SHARED_BUS1: OnceLock<
    Mutex<ThreadModeRawMutex, I2c<'static, Async, i2c::mode::Master>>,
> = OnceLock::new();
pub static SHARED_BUS2: OnceLock<
    Mutex<ThreadModeRawMutex, I2c<'static, Async, i2c::mode::Master>>,
> = OnceLock::new();

pub static SPI_BUS1: OnceLock<Mutex<ThreadModeRawMutex, Spi<'static, Async, spi::mode::Master>>> =
    OnceLock::new();
pub static SPI_BUS3: OnceLock<Mutex<ThreadModeRawMutex, Spi<'static, Async, spi::mode::Master>>> =
    OnceLock::new();

pub static USART1: OnceLock<Mutex<ThreadModeRawMutex, Uart<'static, Async>>> = OnceLock::new();
pub static USART3: OnceLock<Mutex<ThreadModeRawMutex, Uart<'static, Async>>> = OnceLock::new();

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SensorId {
    /// MS5607 barometer on I2C2 (shared bus 1: PA9 = SCL, PA8 = SDA)
    BarometerBus1,
    /// MS5607 barometer on I2C3 (shared bus 2: PC8 = SCL, PC9 = SDA)
    BarometerBus2,
    /// `SHT4x` digital temperature & humidity sensor on I2C2 (shared bus 1)
    DhtBus1,
    /// `SHT4x` digital temperature & humidity sensor on I2C3 (shared bus 2)
    DhtBus2,
    /// LSM6DSO32 accelerometer/gyroscope on SPI3 (PB3 = SCK, PB4 = MISO, PB5 = MOSI, PB6 = CS, PC13 = INT)
    Imu1,
    /// LSM6DSO32 accelerometer/gyroscope on SPI1 (PA5 = SCK, PA6 = MISO, PA7 = MOSI, PA4 = CS, PC14 = INT)
    Imu2,
    /// LSM303AGR magnetometer/accelerometer on I2C2 (shared bus 1) with PA0 as INT
    MagnetometerBus1,
    /// LSM303AGR magnetometer/accelerometer on I2C3 (shared bus 2) with PA1 as INT
    MagnetometerBus2,
    /// GPS module 1 connected to USART1 (PC5 = TX, PC4 = RX)
    Gps1,
    /// GPS module 2 connected to USART3 (PC11 = TX, PC10 = RX)
    Gps2,
}

/// Configuration parameters for the sensor behaviour.
#[derive(Debug, Clone, Copy)]
pub struct CommonSensorConfig {
    /// Maximum allowed consecutive errors before restarting the sensor.
    max_consecutive_errors: u8,
    /// Base delay (in milliseconds) for exponential backoff during initialization.
    base_backoff_ms: u32,
    /// Maximum delay (in milliseconds) for exponential backoff during initialization.
    max_backoff_ms: u32,
}

/// An atomic sensor status with two states: Inactive and Active.
#[atomic_enum::atomic_enum]
pub enum SensorStatus {
    Inactive,
    Active,
}

impl From<SensorStatus> for hermes_can::messages::board_status::SensorStatus {
    fn from(value: SensorStatus) -> Self {
        match value {
            SensorStatus::Inactive => hermes_can::messages::board_status::SensorStatus::Offline,
            SensorStatus::Active => hermes_can::messages::board_status::SensorStatus::Online,
        }
    }
}

pub static BAROMETER_BUS_1_STATUS: AtomicSensorStatus =
    AtomicSensorStatus::new(SensorStatus::Inactive);
pub static BAROMETER_BUS_2_STATUS: AtomicSensorStatus =
    AtomicSensorStatus::new(SensorStatus::Inactive);
pub static DHT_BUS_1_STATUS: AtomicSensorStatus = AtomicSensorStatus::new(SensorStatus::Inactive);

pub static DHT_BUS_2_STATUS: AtomicSensorStatus = AtomicSensorStatus::new(SensorStatus::Inactive);
pub static IMU1_STATUS: AtomicSensorStatus = AtomicSensorStatus::new(SensorStatus::Inactive);
pub static IMU2_STATUS: AtomicSensorStatus = AtomicSensorStatus::new(SensorStatus::Inactive);
pub static MAGNETOMETER_BUS_1_STATUS: AtomicSensorStatus =
    AtomicSensorStatus::new(SensorStatus::Inactive);
pub static MAGNETOMETER_BUS_2_STATUS: AtomicSensorStatus =
    AtomicSensorStatus::new(SensorStatus::Inactive);
pub static GPS1_STATUS: AtomicSensorStatus = AtomicSensorStatus::new(SensorStatus::Inactive);
pub static GPS2_STATUS: AtomicSensorStatus = AtomicSensorStatus::new(SensorStatus::Inactive);

pub fn update_status(status: SensorStatus, id: SensorId) {
    match id {
        SensorId::BarometerBus1 => BAROMETER_BUS_1_STATUS.store(status, Ordering::Relaxed),
        SensorId::BarometerBus2 => BAROMETER_BUS_2_STATUS.store(status, Ordering::Relaxed),
        SensorId::DhtBus1 => DHT_BUS_1_STATUS.store(status, Ordering::Relaxed),
        SensorId::DhtBus2 => DHT_BUS_2_STATUS.store(status, Ordering::Relaxed),
        SensorId::Imu1 => IMU1_STATUS.store(status, Ordering::Relaxed),
        SensorId::Imu2 => IMU2_STATUS.store(status, Ordering::Relaxed),
        SensorId::MagnetometerBus1 => MAGNETOMETER_BUS_1_STATUS.store(status, Ordering::Relaxed),
        SensorId::MagnetometerBus2 => MAGNETOMETER_BUS_2_STATUS.store(status, Ordering::Relaxed),
        SensorId::Gps1 => GPS1_STATUS.store(status, Ordering::Relaxed),
        SensorId::Gps2 => GPS2_STATUS.store(status, Ordering::Relaxed),
    }
}
