//! Board address from the ADD0..ADD3 solder jumpers. A bridged jumper ties its
//! pin to 3V3 and there are no external resistors, so open pins rely on the
//! internal pull-downs and read as 0.

use embassy_stm32::gpio::{Input, Pull};
use embassy_time::Timer;

use super::Address;

impl Address {
    /// The address in 0..=15, ADD0 being the least significant bit.
    pub async fn read(self) -> u8 {
        let pins = [
            Input::new(self.add0, Pull::Down),
            Input::new(self.add1, Pull::Down),
            Input::new(self.add2, Pull::Down),
            Input::new(self.add3, Pull::Down),
        ];
        // Let the open pins discharge through the pull-downs.
        Timer::after_micros(10).await;

        let mut address = 0;
        for (bit, pin) in pins.iter().enumerate() {
            if pin.is_high() {
                address |= 1 << bit;
            }
        }
        address
    }
}
