use embassy_stm32::gpio::{Level, Output, Speed};

use super::{GreenLed, RedLed, YellowLed};

pub struct BoardLeds {
    pub green: Output<'static>,
    pub yellow: Output<'static>,
    pub red: Output<'static>,
}

impl BoardLeds {
    pub fn setup(green: GreenLed, yellow: YellowLed, red: RedLed) -> Self {
        Self {
            green: Output::new(green.pin, Level::Low, Speed::Low),
            yellow: Output::new(yellow.pin, Level::Low, Speed::Low),
            red: Output::new(red.pin, Level::Low, Speed::Low),
        }
    }
}
