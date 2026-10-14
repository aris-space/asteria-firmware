//! Bridged jumpers tie their pin to 3V3 and open ones float, hence the pull-downs.

use embassy_stm32::gpio::{Input, Pull};
use embassy_time::Timer;

use super::Address;

impl Address {
    pub async fn read(self) -> u8 {
        let pins = [
            Input::new(self.add0, Pull::Down),
            Input::new(self.add1, Pull::Down),
            Input::new(self.add2, Pull::Down),
            Input::new(self.add3, Pull::Down),
        ];
        // Let open pins settle.
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
