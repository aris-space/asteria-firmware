use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals::TIM4;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};

use super::Buzzer;

pub type BuzzerPwm = SimplePwm<'static, TIM4>;

impl Buzzer {
    pub fn setup(self) -> BuzzerPwm {
        let pin = PwmPin::new(self.pin, OutputType::PushPull);
        SimplePwm::new(
            self.timer,
            None,
            None,
            None,
            Some(pin),
            Hertz(3_000),
            CountingMode::EdgeAlignedUp,
        )
    }
}
