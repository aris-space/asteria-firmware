//! Audible status on the piezo: a chime at start-up, and a chirp when the
//! board gains or loses a 3D GNSS fix on either receiver.

use embassy_stm32::time::Hertz;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer, with_timeout};

use crate::resources::buzzer::BuzzerPwm;
use crate::sensors::GNSS_COUNT;
use crate::signals;

// A change of fix is announced only after it held this long, so a flickering
// fix does not chirp repeatedly.
const SETTLE: Duration = Duration::from_secs(2);
// A receiver whose last 3D fix arrived longer ago than this has no fix.
const FIX_STALE: Duration = Duration::from_secs(1);

#[derive(Clone, Copy)]
enum Step {
    Tone(Hertz, Duration),
    Rest(Duration),
}

const fn tone(hz: u32, ms: u64) -> Step {
    Step::Tone(Hertz(hz), Duration::from_millis(ms))
}

const fn rest(ms: u64) -> Step {
    Step::Rest(Duration::from_millis(ms))
}

/// Rising fifths and octaves, A5 E6 A6 E7.
const STARTUP: &[Step] = &[
    tone(880, 90),
    rest(30),
    tone(1_319, 90),
    rest(30),
    tone(1_760, 90),
    rest(30),
    tone(2_637, 220),
];
/// Two quick rising chirps, C7 E7.
const FIX_ACQUIRED: &[Step] = &[tone(2_093, 60), rest(40), tone(2_637, 140)];
/// A falling pair, E7 G6.
const FIX_LOST: &[Step] = &[tone(2_637, 60), rest(40), tone(1_568, 200)];

#[embassy_executor::task]
pub async fn task(mut pwm: BuzzerPwm) -> ! {
    play(&mut pwm, STARTUP).await;

    let mut gnss = signals::GNSS_CHANNEL
        .subscriber()
        .expect("buzzer: subscriber slot");
    let mut last_fix: [Option<Instant>; GNSS_COUNT] = [None; GNSS_COUNT];
    let mut announced = false;
    let mut changed_since: Option<Instant> = None;
    loop {
        if let Ok(WaitResult::Message(reading)) = with_timeout(FIX_STALE, gnss.next_message()).await
            && reading.cal.pvt.has_3d_fix()
        {
            last_fix[reading.cal.src.index()] = Some(Instant::now());
        }
        let now = Instant::now();
        let fixed = last_fix
            .iter()
            .flatten()
            .any(|&at| now.saturating_duration_since(at) < FIX_STALE);
        if fixed == announced {
            changed_since = None;
            continue;
        }
        let since = *changed_since.get_or_insert(now);
        if now.saturating_duration_since(since) >= SETTLE {
            play(&mut pwm, if fixed { FIX_ACQUIRED } else { FIX_LOST }).await;
            announced = fixed;
            changed_since = None;
        }
    }
}

async fn play(pwm: &mut BuzzerPwm, steps: &[Step]) {
    for &step in steps {
        match step {
            Step::Tone(frequency, duration) => {
                pwm.set_frequency(frequency);
                // A passive piezo is loudest at 50 % duty.
                let half = pwm.ch4().max_duty_cycle() / 2;
                pwm.ch4().set_duty_cycle(half);
                pwm.ch4().enable();
                Timer::after(duration).await;
                pwm.ch4().disable();
            }
            Step::Rest(duration) => Timer::after(duration).await,
        }
    }
}
