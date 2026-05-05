use crate::sensor_readout::RAIL_24V_LAST;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::GeneralInstance4Channel;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};
use micromath::F32Ext;

#[embassy_executor::task]
pub async fn buzzer_task(pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM2>) {
    let mut buzzer = Buzzer::new(pwm);
    buzzer.set_volume(100.0);
    buzzer.play_sequence(scripts::STARTUP, 1).await;

    let mut rail_24v = RAIL_24V_LAST
        .receiver()
        .expect("Failed to get 24V rail receiver");

    const EXTERNAL_POWER_CONNECT_VOLTAGE: f32 = 25.5;
    const EXTERNAL_POWER_DISCONNECT_VOLTAGE: f32 = 25.1;
    const THRESHOLD_WARNING: f32 = 22.2;
    const THRESHOLD_EXTREME: f32 = 21.0;

    // wait for initial measurement
    let initial_readout = rail_24v.get().await;
    let mut prev_voltage = initial_readout.data.voltage;

    let mut initialisation = true;
    loop {
        let voltage = rail_24v
            .try_get()
            .expect("This should never fail, since we wait on initial readout")
            .data
            .voltage;

        if voltage >= EXTERNAL_POWER_CONNECT_VOLTAGE
            && (prev_voltage < EXTERNAL_POWER_CONNECT_VOLTAGE || initialisation)
        {
            buzzer
                .play_sequence(scripts::CONNECT_EXTERNAL_POWER, 1)
                .await;
        } else if voltage < EXTERNAL_POWER_DISCONNECT_VOLTAGE
            && (prev_voltage >= EXTERNAL_POWER_DISCONNECT_VOLTAGE)
        {
            buzzer
                .play_sequence(scripts::DISCONNECT_EXTERNAL_POWER, 1)
                .await;
        } else if voltage < THRESHOLD_EXTREME {
            buzzer.play_sequence(scripts::EXTREME_WARNING, 1).await;
        } else if voltage < THRESHOLD_WARNING {
            buzzer.play_sequence(scripts::WARNING, 1).await;
        } else {
            Timer::after(Duration::from_millis(100)).await;
        }

        initialisation = false;
        prev_voltage = voltage;
    }
}

/// One item in a buzzer script.
#[derive(Copy, Clone)]
pub enum Step {
    /// Play `freq` for `dur` using the buzzer’s current volume.
    Tone(Hertz, Duration),
    /// Silence for `dur`.
    Wait(Duration),
}

/// Async helper for a piezo / speaker on TIM channel 3.
///
/// *Global* loudness is still handled by `set_volume`.
pub struct Buzzer<Tim: GeneralInstance4Channel> {
    pwm: SimplePwm<'static, Tim>,
    max_duty: u32,
    duty: u32,
    /// 0.0–100.0 %
    volume: f32,
}

#[allow(dead_code)]
impl<Tim: GeneralInstance4Channel> Buzzer<Tim> {
    pub fn new(mut pwm: SimplePwm<'static, Tim>) -> Self {
        let max = pwm.ch3().max_duty_cycle();
        pwm.ch3().disable();
        Self {
            pwm,
            max_duty: max,
            duty: max / 2,
            volume: 50.0,
        }
    }

    pub fn set_volume(&mut self, vol: f32) {
        // Clamp to a valid percentage and guard against NaN/inf input.
        self.volume = if vol.is_finite() {
            vol.clamp(0.0, 100.0)
        } else {
            0.0
        };
        let duty = (self.max_duty as f32 * (self.volume / 100.0)).round() as u32;
        self.duty = duty.min(self.max_duty);
    }

    fn duty_for_current_frequency(&mut self) -> u32 {
        let max = self.pwm.ch3().max_duty_cycle();
        self.duty.min(max)
    }
    pub fn set_tone(&mut self, f: Hertz) {
        self.pwm.set_frequency(f);
        let duty = self.duty_for_current_frequency();
        self.pwm.ch3().set_duty_cycle(duty);
        self.pwm.ch3().enable();
    }

    pub fn stop(&mut self) {
        self.pwm.ch3().disable();
    }
    pub async fn play_tone(&mut self, f: Hertz, d: Duration) {
        self.pwm.set_frequency(f);
        let duty = self.duty_for_current_frequency();
        self.pwm.ch3().set_duty_cycle(duty);
        self.pwm.ch3().enable();
        Timer::after(d).await;
        self.pwm.ch3().disable();
    }
    pub async fn play_sequence(&mut self, script: &[Step], repeats: usize) {
        for _ in 0..repeats {
            for s in script {
                match *s {
                    Step::Tone(f, d) => self.play_tone(f, d).await,
                    Step::Wait(d) => Timer::after(d).await,
                }
            }
        }
    }
}

pub mod scripts {
    #![allow(dead_code)]
    use crate::buzzer::Step;
    use embassy_stm32::time::Hertz;
    use embassy_time::Duration;

    // ----------- WARNING SOUNDS ----------- //
    // ordered by increasing urgency

    /// Caution sound, best when played in a loop
    pub const CAUTION: &[Step] = &[
        Step::Tone(Hertz(740), Duration::from_millis(200)),
        Step::Tone(Hertz(738), Duration::from_millis(200)),
        Step::Wait(Duration::from_millis(400)),
    ];

    /// Low fast beep sound, played in a loop
    pub const LOW_FAST: &[Step] = &[
        Step::Tone(Hertz(587), Duration::from_millis(500)),
        Step::Wait(Duration::from_millis(30)),
    ];

    /// Warning sound, best when played in a loop.
    pub const WARNING: &[Step] = &[
        Step::Tone(Hertz(450 + 550), Duration::from_millis(15)),
        Step::Tone(Hertz(440 + 550), Duration::from_millis(15)),
        Step::Tone(Hertz(430 + 550), Duration::from_millis(15)),
        Step::Tone(Hertz(420 + 550), Duration::from_millis(15)),
        Step::Tone(Hertz(410 + 550), Duration::from_millis(15)),
        Step::Tone(Hertz(400 + 550), Duration::from_millis(15)),
        Step::Tone(Hertz(440 + 550), Duration::from_millis(300)),
    ];

    /// Extreme warning sound, best when played in a loop
    pub const EXTREME_WARNING: &[Step] = &[
        Step::Tone(Hertz(1190), Duration::from_millis(300)),
        Step::Tone(Hertz(345), Duration::from_millis(300)),
    ];

    // ----------- ACKNOWLEDGEMENTS ----------- //

    /// Acknowledge sound, meant to be played once.
    pub const ACKNOWLEDGE: &[Step] = &[
        Step::Tone(Hertz(1000), Duration::from_millis(100)),
        Step::Wait(Duration::from_millis(100)),
        Step::Tone(Hertz(1000), Duration::from_millis(100)),
        Step::Wait(Duration::from_millis(100)),
    ];

    pub const ACKNOWLEDGE_UP: &[Step] = &[
        Step::Tone(Hertz(659), Duration::from_millis(100)),
        Step::Wait(Duration::from_millis(100)),
        Step::Tone(Hertz(1000), Duration::from_millis(100)),
        Step::Wait(Duration::from_millis(100)),
    ];

    pub const ACKNOWLEDGE_DOWN: &[Step] = &[
        Step::Tone(Hertz(1000), Duration::from_millis(100)),
        Step::Wait(Duration::from_millis(100)),
        Step::Tone(Hertz(659), Duration::from_millis(100)),
        Step::Wait(Duration::from_millis(100)),
    ];

    // ----------- OTHER SOUNDS ----------- //

    /// Startup melody, meant to be played once.
    pub const STARTUP: &[Step] = &[
        Step::Tone(Hertz(523), Duration::from_millis(80)), // C5
        Step::Wait(Duration::from_millis(50)),
        Step::Tone(Hertz(659), Duration::from_millis(80)), // E5
        Step::Wait(Duration::from_millis(50)),
        Step::Tone(Hertz(784), Duration::from_millis(120)), // G5
        Step::Wait(Duration::from_millis(50)),
    ];

    /// Urgent notification sound, sound good when played 3 times in sequence
    pub const URGENT: &[Step] = &[
        Step::Tone(Hertz(700), Duration::from_millis(80)),
        Step::Tone(Hertz(1000), Duration::from_millis(80)),
        Step::Tone(Hertz(1400), Duration::from_millis(80)),
        Step::Wait(Duration::from_millis(160)),
    ];

    /// Connect external power sound, meant to be played once.
    pub const CONNECT_EXTERNAL_POWER: &[Step] = &[
        Step::Tone(Hertz(440), Duration::from_millis(60)),
        Step::Wait(Duration::from_millis(40)),
        Step::Tone(Hertz(554), Duration::from_millis(60)),
        Step::Wait(Duration::from_millis(40)),
        Step::Tone(Hertz(659), Duration::from_millis(60)),
        Step::Wait(Duration::from_millis(40)),
        Step::Tone(Hertz(880), Duration::from_millis(400)),
        Step::Wait(Duration::from_millis(200)),
        Step::Tone(Hertz(880), Duration::from_millis(400)),
        Step::Wait(Duration::from_millis(200)),
        Step::Tone(Hertz(880), Duration::from_millis(400)),
    ];

    /// Discconnect external power sound, meant to be played once.
    pub const DISCONNECT_EXTERNAL_POWER: &[Step] = &[
        Step::Tone(Hertz(880), Duration::from_millis(60)),
        Step::Wait(Duration::from_millis(40)),
        Step::Tone(Hertz(659), Duration::from_millis(60)),
        Step::Wait(Duration::from_millis(40)),
        Step::Tone(Hertz(554), Duration::from_millis(60)),
        Step::Wait(Duration::from_millis(40)),
        Step::Tone(Hertz(440), Duration::from_millis(400)),
        Step::Wait(Duration::from_millis(200)),
        Step::Tone(Hertz(440), Duration::from_millis(400)),
        Step::Wait(Duration::from_millis(200)),
        Step::Tone(Hertz(440), Duration::from_millis(400)),
    ];
}
