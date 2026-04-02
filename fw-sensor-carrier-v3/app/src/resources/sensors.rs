use super::*;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::interrupt::typelevel;
use embassy_stm32::mode::Async;
use embassy_stm32::spi;
use embassy_stm32::spi::Spi;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use lsm6dso32::spi::Lsm6Dso32SpiInterface;

type SharedSpiDevice = ExclusiveDevice<Spi<'static, Async, SpiMaster>, Output<'static>, Delay>;
pub type ImuInterface = Lsm6Dso32SpiInterface<SharedSpiDevice>;


impl Imu1 {
    pub fn setup(self, config: spi::Config) -> (ImuInterface, ExtiInput<'static, Async>) {
        bind_interrupts!(struct Imu1Irqs {
            DMA1_STREAM4 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH4>;
            DMA1_STREAM5 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH5>;
            EXTI4 => exti::InterruptHandler<typelevel::EXTI4>;
        });
        let spi = Spi::new(
            self.periph,
            self.sck,
            self.mosi,
            self.miso,
            self.tx_dma,
            self.rx_dma,
            Imu1Irqs,
            config,
        );
        let cs = Output::new(self.cs, Level::High, Speed::VeryHigh);
        let int1 = ExtiInput::new(self.int1, self.exti, Pull::None, Imu1Irqs);
        let spi = ExclusiveDevice::new(spi, cs, Delay)
            .expect("Error while creating exclusive device. CS pin set failed.");

        (Lsm6Dso32SpiInterface { spi }, int1)
    }
}

impl Imu2 {
    pub fn setup(self, config: spi::Config) -> (ImuInterface, ExtiInput<'static, Async>) {
        bind_interrupts!(struct Imu2Irqs {
            DMA1_STREAM6 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH6>;
            DMA1_STREAM7 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH7>;
            EXTI15_10 => exti::InterruptHandler<typelevel::EXTI15_10>;
        });
        let spi = Spi::new(
            self.periph,
            self.sck,
            self.mosi,
            self.miso,
            self.tx_dma,
            self.rx_dma,
            Imu2Irqs,
            config,
        );
        let cs = Output::new(self.cs, Level::High, Speed::VeryHigh);
        let int1 = ExtiInput::new(self.int1, self.exti, Pull::None, Imu2Irqs);
        let spi = ExclusiveDevice::new(spi, cs, Delay)
            .expect("Error while creating exclusive device. CS pin set failed.");

        (Lsm6Dso32SpiInterface { spi }, int1)
    }
}
