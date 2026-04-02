use crate::resources::I2cMaster;
use embassy_stm32::{bind_interrupts, i2c, peripherals};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use static_cell::StaticCell;

use super::{Bus1, Bus2};

pub type SharedI2c = embassy_stm32::i2c::I2c<'static, Async, I2cMaster>;
pub type SharedI2cBus = &'static Mutex<CriticalSectionRawMutex, SharedI2c>;

static SHARED_I2C_BUS_1: StaticCell<Mutex<CriticalSectionRawMutex, SharedI2c>> = StaticCell::new();
static SHARED_I2C_BUS_2: StaticCell<Mutex<CriticalSectionRawMutex, SharedI2c>> = StaticCell::new();

impl Bus1 {
    pub fn setup(self, frequency: embassy_stm32::time::Hertz, config: i2c::Config) -> SharedI2c {
        bind_interrupts!(struct Bus1Irqs {
            I2C5_EV => i2c::EventInterruptHandler<peripherals::I2C5>;
            I2C5_ER => i2c::ErrorInterruptHandler<peripherals::I2C5>;
            DMA2_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH2>;
            DMA2_STREAM7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH7>;
        });
        let mut config = config;
        config.frequency = frequency;

        i2c::I2c::new(
            self.periph,
            self.scl,
            self.sda,
            self.tx_dma,
            self.rx_dma,
            Bus1Irqs,
            config,
        )
    }

    pub fn setup_shared(
        self,
        frequency: embassy_stm32::time::Hertz,
        config: i2c::Config,
    ) -> SharedI2cBus {
        SHARED_I2C_BUS_1.init(Mutex::new(self.setup(frequency, config)))
    }
}

impl Bus2 {
    pub fn setup(self, frequency: embassy_stm32::time::Hertz, config: i2c::Config) -> SharedI2c {
        bind_interrupts!(struct Bus2Irqs {
            I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
            I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
            DMA2_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH0>;
            DMA2_STREAM1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH1>;
        });
        let mut config = config;
        config.frequency = frequency;

        i2c::I2c::new(
            self.periph,
            self.scl,
            self.sda,
            self.tx_dma,
            self.rx_dma,
            Bus2Irqs,
            config,
        )
    }

    pub fn setup_shared(
        self,
        frequency: embassy_stm32::time::Hertz,
        config: i2c::Config,
    ) -> SharedI2cBus {
        SHARED_I2C_BUS_2.init(Mutex::new(self.setup(frequency, config)))
    }
}
