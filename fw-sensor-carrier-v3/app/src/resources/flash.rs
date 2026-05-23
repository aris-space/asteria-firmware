use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::spi::Spi;
use embassy_stm32::spi::mode::Master as SpiMaster;
use embassy_stm32::{bind_interrupts, peripherals, spi};
use embassy_time::Delay;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use static_cell::StaticCell;
use w25q256jv::W25q256jv;

use super::Flash;

type FlashSpi = Spi<'static, Async, SpiMaster>;
type FlashDevice = ExclusiveDevice<FlashSpi, Output<'static>, Delay>;

pub type BoardFlash = W25q256jv<FlashDevice, FixedHighPin, FixedHighPin>;

static BOARD_FLASH: StaticCell<BoardFlash> = StaticCell::new();

fn config() -> spi::Config {
    let mut config = spi::Config::default();
    config.frequency = embassy_stm32::time::mhz(50);
    config.gpio_speed = Speed::VeryHigh;
    config
}

impl Flash {
    pub fn setup(self) -> &'static mut BoardFlash {
        bind_interrupts!(struct FlashIrqs {
            DMA2_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH3>;
            DMA2_STREAM4 => embassy_stm32::dma::InterruptHandler<peripherals::DMA2_CH4>;
        });

        let spi = Spi::new(
            self.periph,
            self.sck,
            self.mosi,
            self.miso,
            self.tx_dma,
            self.rx_dma,
            FlashIrqs,
            config(),
        );
        let cs = Output::new(self.cs, Level::High, Speed::VeryHigh);
        let device = ExclusiveDevice::new(spi, cs, Delay).expect("flash spi exclusive");
        let flash = W25q256jv::new(device, FixedHighPin, FixedHighPin).expect("w25q256jv init");
        BOARD_FLASH.init(flash)
    }
}

/// HOLD/WP are tied high in hardware; the driver still wants `OutputPin`s,
/// so we hand it no-op pins that pretend they are always driven high.
pub struct FixedHighPin;

impl embedded_hal::digital::ErrorType for FixedHighPin {
    type Error = core::convert::Infallible;
}

impl OutputPin for FixedHighPin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
