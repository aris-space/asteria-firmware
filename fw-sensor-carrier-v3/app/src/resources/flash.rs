#[cfg(feature = "storage")]
mod imp {
    use embassy_stm32::gpio::{Level, Output, Speed};
    use embassy_stm32::mode::Blocking;
    use embassy_stm32::spi::Spi;
    use embassy_stm32::spi::mode::Master as SpiMaster;
    use embassy_time::Delay;
    use embedded_hal::digital::OutputPin;
    use embedded_hal_bus::spi::ExclusiveDevice;
    use static_cell::StaticCell;
    use w25q256jv::W25q256jv;

    use crate::resources::Flash;

    type FlashSpi = Spi<'static, Blocking, SpiMaster>;
    pub type FlashDevice = ExclusiveDevice<FlashSpi, Output<'static>, Delay>;
    pub type BoardFlash = W25q256jv<FlashDevice, FixedHighPin, FixedHighPin>;

    static BOARD_FLASH: StaticCell<W25q256jv<FlashDevice, FixedHighPin, FixedHighPin>> =
        StaticCell::new();

    fn config() -> embassy_stm32::spi::Config {
        let mut config = embassy_stm32::spi::Config::default();
        config.frequency = embassy_stm32::time::mhz(50);
        config.gpio_speed = Speed::VeryHigh;
        config
    }

    impl Flash {
        pub fn setup(self) -> &'static mut BoardFlash {
            let spi = Spi::new_blocking(self.periph, self.sck, self.mosi, self.miso, config());
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
}

#[cfg(not(feature = "storage"))]
mod imp {
    use crate::resources::Flash;
    use static_cell::StaticCell;

    pub type BoardFlash = ();

    static BOARD_FLASH: StaticCell<()> = StaticCell::new();

    impl Flash {
        pub fn setup(self) -> &'static mut BoardFlash {
            let _ = self;
            BOARD_FLASH.init(())
        }
    }
}

pub use imp::*;
