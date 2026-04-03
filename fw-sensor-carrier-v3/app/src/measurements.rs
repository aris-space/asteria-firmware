use embassy_time::{Duration, Instant};
use lsm6dso32::types::{Acceleration, AngularRate};

use crate::sensors::{BarometerId, ImuId};

#[derive(Clone, Copy, Debug)]
pub struct Timestamped<T> {
    pub ts: Instant,
    pub value: T,
}

impl<T> Timestamped<T> {
    pub fn new(ts: Instant, value: T) -> Self {
        Self { ts, value }
    }

    pub fn now(value: T) -> Self {
        Self::new(Instant::now(), value)
    }

    pub fn now_with_delay(value: T, delay: Duration) -> Self {
        Self::new(Instant::now() - delay, value)
    }

    pub fn at(ts: Instant, value: T) -> Self {
        Self::new(ts, value)
    }
}

#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub sensor_id: ImuId,
    pub data: Timestamped<ImuData>,
}

#[derive(Clone, Copy, Debug)]
pub struct ImuData {
    pub accel: Acceleration,
    pub gyro: AngularRate,
}

#[derive(Clone, Copy, Debug)]
pub struct PressureSample {
    pub sensor_id: BarometerId,
    pub data: Timestamped<PressureData>,
}

#[derive(Clone, Copy, Debug)]
pub struct PressureData {
    pub pressure_mbar: f32,
    pub temperature_c: f32,
}
