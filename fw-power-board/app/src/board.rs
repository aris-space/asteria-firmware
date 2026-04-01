// LTC2945 sensor address (shared on I2C2 and I2C3)
pub const LTC2945_I2C_ADDR: u8 = 0x67;

// Bind I2C interrupts
use embassy_stm32::can;
use embassy_stm32::dma;
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::FDCAN1;
use embassy_stm32::peripherals::{I2C2, I2C3, I2C4};
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_sync::signal::Signal;

bind_interrupts!(pub struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<I2C2>;
    I2C3_EV => i2c::EventInterruptHandler<I2C3>;
    I2C3_ER => i2c::ErrorInterruptHandler<I2C3>;
    I2C4_EV => i2c::EventInterruptHandler<I2C4>;
    I2C4_ER => i2c::ErrorInterruptHandler<I2C4>;
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
    DMA2_CHANNEL1 => dma::InterruptHandler<peripherals::DMA2_CH1>;
    DMA2_CHANNEL2 => dma::InterruptHandler<peripherals::DMA2_CH2>;
    DMA2_CHANNEL3 => dma::InterruptHandler<peripherals::DMA2_CH3>;
    DMA2_CHANNEL4 => dma::InterruptHandler<peripherals::DMA2_CH4>;
});

pub static RECORDING_CAMERA: OnceLock<Mutex<ThreadModeRawMutex, Output>> = OnceLock::new();
pub static LIVESTREAM_CAMERA: OnceLock<Mutex<ThreadModeRawMutex, Output>> = OnceLock::new();
pub static CAMERA_SETTINGS_CHANGED: Signal<ThreadModeRawMutex, ()> = Signal::new();
