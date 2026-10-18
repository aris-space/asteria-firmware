//! Bridged jumpers tie their pin to 3V3 and open ones float, hence the pull-downs.

use embassy_stm32::gpio::{Input, Pull};
use embassy_time::{Duration, block_for};

use super::Address;

impl Address {
    pub fn setup(self) -> u8 {
        let pins = [
            Input::new(self.add0, Pull::Down),
            Input::new(self.add1, Pull::Down),
            Input::new(self.add2, Pull::Down),
            Input::new(self.add3, Pull::Down),
        ];
        // Let open pins settle, ~40k pull-down * ~10 pF is well under 1 us, so 10 us should be plenty.
        block_for(Duration::from_micros(10));

        let mut address = 0;
        for (bit, pin) in pins.iter().enumerate() {
            if pin.is_high() {
                address |= 1 << bit;
            }
        }
        address
    }
}
