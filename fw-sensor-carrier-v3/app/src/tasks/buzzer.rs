//! Audible status indicator on the passive piezo buzzer.
//!
//! Waits on event signals and plays a distinct beep pattern for each:
//!   - calibration complete: two beeps at 2.5 kHz
//!   - first GNSS fix:        three beeps at 3.5 kHz

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Timer};

use crate::resources::buzzer::BuzzerPwm;
use crate::signals;

/// Sound the piezo at `freq` Hz for `ms` ms (50% duty square wave).
async fn beep(buzzer: &mut BuzzerPwm, freq: u32, ms: u64) {
    buzzer.set_frequency(Hertz(freq));
    let half = buzzer.ch4().max_duty_cycle() / 2;
    buzzer.ch4().set_duty_cycle(half);
    buzzer.ch4().enable();
    Timer::after(Duration::from_millis(ms)).await;
    buzzer.ch4().disable();
}

/// Play `count` beeps of `ms` each, separated by `gap_ms` of silence.
async fn pattern(buzzer: &mut BuzzerPwm, freq: u32, ms: u64, count: usize, gap_ms: u64) {
    for i in 0..count {
        if i > 0 {
            Timer::after(Duration::from_millis(gap_ms)).await;
        }
        beep(buzzer, freq, ms).await;
    }
}

#[embassy_executor::task]
pub async fn task(mut buzzer: BuzzerPwm) -> ! {
    loop {
        match select(
            signals::CALIBRATION_DONE.wait(),
            signals::FIRST_GNSS_FIX.wait(),
        )
        .await
        {
            Either::First(()) => {
                info!("buzzer: calibration done");
                pattern(&mut buzzer, 2500, 120, 2, 100).await;
            }
            Either::Second(()) => {
                info!("buzzer: first GNSS fix");
                pattern(&mut buzzer, 3500, 90, 3, 80).await;
            }
        }
    }
}
