use crate::dpr::DprLoopState::{ActiveNominal, ActiveOverPressure, Passive};
use crate::pid::{PID, PIDGain};
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Instant, Timer};

const GAINS: PIDGain = PIDGain {
    p: 1.0,
    i: 0.0,
    d: 8.0,
};

const SAFETY_LIMIT_BARG: f32 = 55.0;

const DPR_TICK_DURATION: Duration = Duration::from_millis(1);

#[derive(Debug, Copy, Clone)]
pub enum DprLoopState {
    ActiveNominal,
    ActiveOverPressure,
    Passive,
}

pub struct DPR<'a> {
    valve_pin: Output<'a>,
    pid: PID,
    pub loop_state: DprLoopState,
    pub setpoint: f32,
    pub pressure: f32,
    pub last_time: Instant,
}

impl DPR {
    pub fn new(valve_pin: Output) -> DPR {
        let pid = PID::new(GAINS);
        let loop_state = Passive;
        let setpoint = 0.0;
        let pressure = 0.0;
        let last_time = Instant::now();

        DPR {
            valve_pin,
            pid,
            loop_state,
            setpoint,
            pressure,
            last_time,
        }
    }

    pub fn update_pressure(&mut self, p: f32) {
        // ToDo: proper handling
        if p.is_finite() {
            self.pressure = p;
        }
    }

    pub fn check_overpressure(&mut self) {
        if self.loop_state == ActiveNominal && self.pressure >= SAFETY_LIMIT_BARG {
            self.loop_state = ActiveOverPressure
        }
    }

    pub fn update_valve_state(&mut self) {
        let loop_state = self.loop_state;
        
        match loop_state {
            ActiveNominal => {
                let elapsed_ms = self.last_time.elapsed().as_millis() as f32;
                let opening_time = self.pid.update(self.pressure, elapsed_ms);
                self.last_time = Instant::now();
                self.open_valve(opening_time)
            }
            ActiveOverPressure => {
                if self.pressure < SAFETY_LIMIT_BARG {
                    self.loop_state = ActiveNominal;
                }
                self.valve_pin.set_low();
            }
            Passive => {
                self.valve_pin.set_low();
            }
        }
    }

    pub async fn open_valve(&mut self, time_ms: f32) {
        self.valve_pin.set_high();
        Timer::after_millis(time_ms as u64).await;
        self.valve_pin.set_low();
    }
    
    pub async fn tick(&mut self) {
        Timer::after(DPR_TICK_DURATION).await;
    }
    
    pub fn enable(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
        self.loop_state = ActiveNominal;
        
        self.pid.reset();
        self.last_time = Instant::now();
    }
    
    pub fn disable(&mut self) {
        self.loop_state = Passive;
    }
}
