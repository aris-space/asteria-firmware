use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::sdmmc::{self, Sdmmc};
use embassy_stm32::{bind_interrupts, peripherals};

use super::SdCard;

pub type Sd = Sdmmc<'static>;

impl SdCard {
    /// Returns the SDMMC peripheral plus the card-detect input and the power
    /// enable. The caller must keep `power` alive while talking to the card.
    pub fn setup(self) -> (Sd, Input<'static>, Output<'static>) {
        bind_interrupts!(struct SdIrqs {
            SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
        });

        // SD_VDD comes from a TPS22918 load switch with an active-high enable
        // (100k pulldown, so off at reset). Hold PD6 high to keep the card powered.
        let power = Output::new(self.power, Level::High, Speed::Low);
        // PD3 is card-detect; polarity is board-specific, so it is read-only here.
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
