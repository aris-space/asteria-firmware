use crate::resources::{Flash, SpiMaster};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::mhz;
use embassy_time::Delay;
use embedded_hal::digital::OutputPin;
use embedded_hal_bus::spi::ExclusiveDevice;
use static_cell::StaticCell;
use w25q256jv::W25q256jv;

type FlashSpi = Spi<'static, Blocking, SpiMaster>;
type FlashDevice = ExclusiveDevice<FlashSpi, Output<'static>, Delay>;
pub type BoardFlash = W25q256jv<FlashDevice, FixedHighPin, FixedHighPin>;

pub static BOARD_FLASH: StaticCell<BoardFlash> = StaticCell::new();

impl Flash {
    pub fn setup(self) -> &'static mut BoardFlash {
        let mut spi_config = embassy_stm32::spi::Config::default();
        spi_config.frequency = mhz(50);
        spi_config.gpio_speed = Speed::VeryHigh;

        let spi = Spi::new_blocking(self.periph, self.sck, self.mosi, self.miso, spi_config);
        let cs = Output::new(self.cs, Level::High, Speed::VeryHigh);
        let device = ExclusiveDevice::new(spi, cs, Delay).expect("spi exclusive");
        let flash = W25q256jv::new(device, FixedHighPin, FixedHighPin).expect("w25q256jv init");
        BOARD_FLASH.init(flash)
    }
}

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
