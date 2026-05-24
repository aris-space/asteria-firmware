use crate::globals::STATE;
use cortex_m::prelude::_embedded_hal_Pwm;
use embassy_stm32::peripherals::TIM3;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel::Ch4;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};

#[derive(Clone)]
#[allow(dead_code)]
pub enum BuzzerState {
    Idle,
    Error,
}
#[embassy_executor::task]
pub async fn buzzer_task(mut pwm: SimplePwm<'static, TIM3>) {
    let mut watcher = STATE.buzzer.receiver().unwrap();

    pwm.enable(Ch4);
    let on = (pwm.get_max_duty() as f32 * 0.95) as u32;
    let off = 0;

    start_up(&mut pwm).await;

    pwm.set_frequency(Hertz(440));

    loop {
        let state = watcher.get().await;

        match state {
            BuzzerState::Error => {
                pwm.set_duty(Ch4, on);
            }
            BuzzerState::Idle => {
                pwm.set_duty(Ch4, off);
            }
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

pub(crate) async fn start_up<'a>(pwm: &mut SimplePwm<'a, TIM3>) {
    pwm.set_duty(Ch4, pwm.get_max_duty() / 2);
    Timer::after(Duration::from_millis(250)).await;
    pwm.set_frequency(Hertz(760));
    Timer::after(Duration::from_millis(250)).await;
    pwm.set_frequency(Hertz(1520));
    Timer::after(Duration::from_millis(500)).await;
    pwm.set_duty(Ch4, 0);
}
