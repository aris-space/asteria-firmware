use crate::globals::STATE;
use cortex_m::prelude::_embedded_hal_Pwm;
use embassy_stm32::peripherals::TIM3;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::Channel::Ch4;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Instant, Timer};

const STATUS_BEEP_INTERVAL: Duration = Duration::from_secs(10);
const ARMING_BEEP_INTERVAL: Duration = Duration::from_secs(5);

#[derive(Clone)]
pub enum BuzzerState {
    Armed,
    Idle,
}
#[embassy_executor::task]
pub async fn buzzer_task(mut pwm: SimplePwm<'static, TIM3>) {
    let mut watcher = STATE.buzzer.receiver().unwrap();

    pwm.enable(Ch4);
    let on = (pwm.get_max_duty() as f32 * 0.95) as u32;

    start_up(&mut pwm).await;

    pwm.set_frequency(Hertz(440));

    let mut status_beep_time = Instant::now();
    loop {
        let state = watcher.get().await;

        match state {
            BuzzerState::Armed => {
                if Instant::now() - status_beep_time >= ARMING_BEEP_INTERVAL {
                    for _ in 0..4 {
                        beep(&mut pwm, on).await;
                    }
                    status_beep_time = Instant::now();
                }
            }
            BuzzerState::Idle => {
                if Instant::now() - status_beep_time >= STATUS_BEEP_INTERVAL {
                    for _ in 0..2 {
                        beep(&mut pwm, on).await;
                    }
                    status_beep_time = Instant::now();
                }
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

async fn beep(pwm: &mut SimplePwm<'_, TIM3>, duty: u32) {
    pwm.set_duty(Ch4, duty);
    Timer::after_millis(100).await;
    pwm.set_duty(Ch4, 0);
    Timer::after_millis(100).await;
}
