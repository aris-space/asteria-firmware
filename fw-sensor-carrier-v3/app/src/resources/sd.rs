use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::sdmmc::{self, Sdmmc};
use embassy_stm32::{bind_interrupts, peripherals};

use super::SdCard;

pub type Sd = Sdmmc<'static>;

impl SdCard {
    pub fn setup(self) -> (Sd, Input<'static>, Output<'static>) {
        bind_interrupts!(struct SdIrqs {
            SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
        });

        // SD_VDD is switched by PD6; must remain enabled while the card is in use.
        let power = Output::new(self.power, Level::High, Speed::Low);
        let detect = Input::new(self.detect, Pull::Up);
        let sdmmc = Sdmmc::new_4bit(
            self.periph,
            SdIrqs,
            self.clk,
            self.cmd,
            self.d0,
            self.d1,
            self.d2,
            self.d3,
            sdmmc::Config::default(),
        );
        (sdmmc, detect, power)
    }
}
