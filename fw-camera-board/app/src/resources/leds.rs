use embassy_stm32::gpio::{Level, Output, Speed};

use super::DebugLed;

impl DebugLed {
    pub fn setup(self) -> Output<'static> {
        Output::new(self.pin, Level::Low, Speed::Low)
    }
}
