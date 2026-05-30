use crate::pid::{PID, PIDGain};
use datatypes::actuator::DPRValve;
use datatypes::status::DprGainInfo;
use datatypes::status::DprLoopInfo::{self, *};
use embassy_stm32::gpio::Output;
#[cfg(any(doc, docsrs))]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as DprRawMutex;
#[cfg(not(any(doc, docsrs)))]
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex as DprRawMutex;
use embassy_sync::watch::{Receiver, Sender};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_utils::info;

const SAFETY_LIMIT_BARG: f32 = 52.0;
const RELAXED_TICK_DURATION: Duration = Duration::from_millis(50);
const CRITICAL_TICK_DURATION: Duration = Duration::from_millis(1);

pub const GAINS: PIDGain = PIDGain {
    p: 16.0,
    i: 0.0,
    d: 0.0,
};

pub const MIN_TIME_MS: f32 = 25.0;
pub const MAX_TIME_MS: f32 = 500.0;

pub struct DPR<'a> {
    valve_pin: Output<'a>,
    pid: PID,
    pub loop_state: DprLoopInfo,
    pub pressure: f32,
    pub last_time: Instant,

    control_receiver: Receiver<'a, DprRawMutex, DPRValve, 5>,
    pressure_receiver: Receiver<'a, DprRawMutex, f32, 5>,
    pid_receiver: Receiver<'a, DprRawMutex, DprGainInfo, 5>,
    status_sender: Sender<'a, DprRawMutex, DprLoopInfo, 5>,
}

impl<'a> DPR<'a> {
    pub fn new(
        valve_pin: Output<'a>,
        control_receiver: Receiver<'a, DprRawMutex, DPRValve, 5>,
        pressure_receiver: Receiver<'a, DprRawMutex, f32, 5>,
        pid_receiver: Receiver<'a, DprRawMutex, DprGainInfo, 5>,
        status_sender: Sender<'a, DprRawMutex, DprLoopInfo, 5>,
    ) -> DPR<'a> {
        let pid = PID::new(GAINS, MIN_TIME_MS, MAX_TIME_MS);
        let loop_state = Passive;
        let pressure = 0.0;
        let last_time = Instant::now();

        DPR {
            valve_pin,
            pid,
            loop_state,
            pressure,
            last_time,

            control_receiver,
            pressure_receiver,
            pid_receiver,
            status_sender,
        }
    }

    pub fn update_state(&mut self) {
        if let Some(cfg) = self.control_receiver.try_changed() {
            match cfg {
                DPRValve::Enabled { setpoint: stp } => {
                    self.enable(stp);
                    self.reset();
                }
                DPRValve::Disabled => {
                    self.disable();
                }
            }
            self.status_sender.send(self.loop_state);
        }
    }

    pub fn update_pid(&mut self) {
        if let Some(gains) = self.pid_receiver.try_changed() {
            self.pid.gain = PIDGain {
                p: gains.p,
                i: gains.i,
                d: gains.d,
            };
            self.pid.min_ms = gains.min_ms;
            self.pid.max_ms = gains.max_ms;
        }
    }

    pub fn update_pressure(&mut self) {
        if let Some(p) = self.pressure_receiver.try_changed() {
            // ToDo: proper handling maybe put in pressure driver
            if p.is_finite() {
                self.pressure = p;
            }
        }
    }

    pub fn handle_overpressure(&mut self) {
        match self.loop_state {
            ActiveNominal => {
                if self.pressure > SAFETY_LIMIT_BARG {
                    self.loop_state = ActiveOverPressure;
                    self.status_sender.send(self.loop_state);
                }
            }
            ActiveOverPressure => {
                if self.pressure < SAFETY_LIMIT_BARG {
                    self.loop_state = ActiveNominal;
                    self.status_sender.send(self.loop_state);
                }
            }
            Passive => {}
        }
    }

    pub async fn step(&mut self) {
        match self.loop_state {
            ActiveNominal => {

                if self.pid.max_ms < self.pid.min_ms {
                    // Testing Opening Times
                    let opening_time = self.pid.min_ms as u64;
                    self.actuate_valve(opening_time).await;

                    self.loop_state = Passive;
                    self.status_sender.send(self.loop_state);
                    Timer::after_millis(500).await;
                    let _ = self.control_receiver.try_changed();

                } else {
                    // Running Control Loop
                    let elapsed_ms = self.last_time.elapsed().as_millis() as f32;
                    self.last_time = Instant::now();
                    let opening_time = self.pid.update(self.pressure, elapsed_ms);
                    self.actuate_valve(opening_time).await;
                }
            }
            _ => {
                self.valve_pin.set_low();
                Timer::after(RELAXED_TICK_DURATION).await;
            }
        }

        let loop_state = match self.loop_state {
            ActiveNominal => "ActiveNominal",
            ActiveOverPressure => "ActiveOverPressure",
            Passive => "Passive",
        };
        info!("[DPR] State: {}, Pressure {}", loop_state, self.pressure);
        Timer::after(RELAXED_TICK_DURATION).await;
    }

    pub async fn actuate_valve(&mut self, time_ms: u64) {
        self.valve_pin.set_high();
        self.mindful_await(time_ms).await;
        self.valve_pin.set_low();
    }

    pub async fn mindful_await(&mut self, time_ms: u64) {
        let _ = with_timeout(Duration::from_millis(time_ms), async {
            loop {
                self.update_pressure();
                if self.pressure > SAFETY_LIMIT_BARG || self.pressure > self.pid.setpoint {
                    return;
                }
                Timer::after(CRITICAL_TICK_DURATION).await;
            }
        })
        .await;
    }

    pub fn enable(&mut self, setpoint: f32) {
        self.pid.setpoint = setpoint;
        self.loop_state = ActiveNominal;
    }

    pub fn disable(&mut self) {
        self.loop_state = Passive;
    }

    pub fn reset(&mut self) {
        self.pid.reset();
        self.last_time = Instant::now();
    }
}
