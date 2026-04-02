use embassy_stm32::gpio::{Level, Output, Speed};

use super::{Buzzer, GreenLed, RedLed, YellowLed};

impl Buzzer {
    pub fn setup(self) -> Output<'static> {
        todo!()
    }
}

impl YellowLed {
    pub fn setup(self) -> Output<'static> {
        Output::new(self.pin, Level::Low, Speed::Low)
    }
}

impl GreenLed {
    pub fn setup(self) -> Output<'static> {
        Output::new(self.pin, Level::Low, Speed::Low)
    }
}

impl RedLed {
    pub fn setup(self) -> Output<'static> {
        Output::new(self.pin, Level::Low, Speed::Low)
    }
}
