use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals::TIM4;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};

use super::Buzzer;

// The buzzer is a passive piezo: it needs a PWM square wave to make sound, not
// a static level. PD15 is TIM4_CH4 on the STM32H723.
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
