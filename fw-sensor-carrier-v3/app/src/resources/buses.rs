use embassy_stm32::i2c::mode::Master as I2cMaster;
use embassy_stm32::mode::Async;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

use super::{Bus1, Bus2};

pub type SharedI2c = embassy_stm32::i2c::I2c<'static, Async, I2cMaster>;
pub type SharedI2cBus = &'static Mutex<CriticalSectionRawMutex, SharedI2c>;

fn config() -> i2c::Config {
    let mut config = i2c::Config::default();
    config.frequency = embassy_stm32::time::khz(100);
    // embassy's async I2C busy-spins (no yield) while waiting for the bus to go
    // idle, so a stuck/shorted bus blocks the shared executor for the full timeout.
    // Keep this just above a real transaction (well under 1 ms at 100 kHz) so one
    // dead bus can't starve the others.
    config.timeout = embassy_time::Duration::from_millis(5);
    config
}

static SHARED_I2C_BUS_1: StaticCell<Mutex<CriticalSectionRawMutex, SharedI2c>> = StaticCell::new();
static SHARED_I2C_BUS_2: StaticCell<Mutex<CriticalSectionRawMutex, SharedI2c>> = StaticCell::new();

impl Bus1 {
    pub fn setup(self) -> SharedI2cBus {
        bind_interrupts!(struct Bus1Irqs {
            I2C5_EV => i2c::EventInterruptHandler<peripherals::I2C5>;
            I2C5_ER => i2c::ErrorInterruptHandler<peripherals::I2C5>;
            DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
            DMA2_STREAM7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH7>;
        });

        let i2c = i2c::I2c::new(
            self.periph,
            self.scl,
            self.sda,
            self.tx_dma,
            self.rx_dma,
            Bus1Irqs,
            config(),
        );
        SHARED_I2C_BUS_1.init(Mutex::new(i2c))
    }
}

impl Bus2 {
    pub fn setup(self) -> SharedI2cBus {
        bind_interrupts!(struct Bus2Irqs {
            I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
            I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
            DMA2_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH0>;
            DMA2_STREAM1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH1>;
        });

        let i2c = i2c::I2c::new(
            self.periph,
            self.scl,
            self.sda,
            self.tx_dma,
            self.rx_dma,
            Bus2Irqs,
            config(),
        );
        SHARED_I2C_BUS_2.init(Mutex::new(i2c))
    }
}
