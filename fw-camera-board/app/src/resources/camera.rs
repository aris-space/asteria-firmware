use embassy_stm32::gpio::{Level, Output, Speed};

use super::CameraPower;

impl CameraPower {
    /// Starts with the camera off. Driving the pin also releases the pull-up
    /// PA15 has after reset, which would otherwise hold EN_CAMERA between on
    /// and off against the external pull-down.
    pub fn setup(self) -> Output<'static> {
        Output::new(self.pin, Level::Low, Speed::Low)
    }
}
