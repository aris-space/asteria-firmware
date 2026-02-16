const SAFETY_LIMIT: f32 = 50.0;
pub const CONTROL_TIME_STEP_MS: f32 = 20.0;
const MAX_INTEGRAL_ERROR: f32 = 20.0;

pub struct PIDGains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
}

pub struct PIDController {
    error_i: f32,
    error_d: f32,
    last_error: f32,
    gains: PIDGains,
}

impl PIDController {
    pub fn new(gains: PIDGains) -> Self {
        PIDController {
            error_i: 0.0,
            error_d: 0.0,
            last_error: 0.0,
            gains,
        }
    }

    pub fn step(&mut self, setpoint: f32, measurement: f32) -> f32 {
        let error = setpoint - measurement;
        self.error_i = (self.error_i + error * (CONTROL_TIME_STEP_MS / 1000.0))
            .clamp(-MAX_INTEGRAL_ERROR, MAX_INTEGRAL_ERROR);
        self.error_d = (error - self.last_error) / (CONTROL_TIME_STEP_MS / 1000.0);
        self.last_error = error;

        let output =
            self.gains.kp * error + self.gains.ki * self.error_i + self.gains.kd * self.error_d;
        output.clamp(0.0, SAFETY_LIMIT)
    }
}
