pub struct PIDGain {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

pub struct PIDError {
    p: f32,
    i: f32,
    d: f32,
    prev_p: f32,
}

pub struct PID {
    gain: PIDGain,
    error: PIDError,
    setpoint: f32,
}

impl PID {
    pub fn new(gain: PIDGain) -> PID {
        let error = PIDError {
            p: 0.0,
            i: 0.0,
            d: 0.0,
            prev_p: 0.0,
        };

        let setpoint = 0.0;

        PID {
            gain,
            error,
            setpoint,
        }
    }

    pub fn update(&mut self, pressure: f32, elapsed_time_s: f32) -> f32 {
        self.error.p = self.setpoint - pressure;
        self.error.i += self.error.p * elapsed_time_s;
        self.error.d = (self.error.p - self.error.prev_p) / elapsed_time_s;
        self.error.prev_p = self.error.p;

        return self.gain.p * self.error.p
            + self.gain.i * self.error.i
            + self.gain.d * self.error.d;
    }

    pub fn reset(&mut self) {
        self.error.p = 0.0;
        self.error.i = 0.0;
        self.error.d = 0.0;
        self.error.prev_p = 0.0;
    }
}
