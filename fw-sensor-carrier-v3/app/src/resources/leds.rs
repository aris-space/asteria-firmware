use embassy_stm32::gpio::{Level, Output, Speed};

use super::YellowLed;

impl YellowLed {
    pub fn setup(self) -> Output<'static> {
        Output::new(self.pin, Level::Low, Speed::Low)
    }
}
