#![allow(dead_code)]

use embassy_time::{Duration, Instant};
use lsm6dso32::types::{Acceleration, AngularRate};

use crate::sensors::{BarometerId, DhtId, GnssId, ImuId, MagnetometerId};

#[derive(Clone, Copy, Debug)]
pub struct Timestamped<T> {
    pub ts: Instant,
    pub value: T,
}

impl<T> Timestamped<T> {
    pub fn new(ts: Instant, value: T) -> Self {
        Self { ts, value }
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

#[derive(Clone, Copy, Debug)]
pub struct GnssSample {
    pub sensor_id: GnssId,
    pub data: Timestamped<PvtData>,
}

/// Raw magnetometer sample (sensor-frame, raw counts).
///
/// Board-frame axis flips and nT scaling happen in the processing task.
#[derive(Clone, Copy, Debug)]
pub struct MagSample {
    pub sensor_id: MagnetometerId,
    pub data: Timestamped<MagData>,
}

#[derive(Clone, Copy, Debug)]
pub struct MagData {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

#[derive(Clone, Copy, Debug)]
pub struct EnvSample {
    pub sensor_id: DhtId,
    pub data: Timestamped<EnvData>,
}

#[derive(Clone, Copy, Debug)]
pub struct EnvData {
    pub temperature_c: f32,
    pub humidity_rh: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct PvtData {
    pub lon_deg: f64,
    pub lat_deg: f64,
    pub fix_type: ublox::GpsFix,
    pub height_msl: f32,
    pub num_satellites: u8,
    pub heading_deg: f32,
    pub heading_accuracy_estimate: f32,
    pub heading_of_vehicle_deg: f32,
    pub vel_north: f32,
    pub vel_east: f32,
    pub vel_down: f32,
    pub pdop: u16,
    pub vert_accuracy: u32,
    pub horiz_accuracy: u32,
    pub magnetic_declination_deg: f32,
    pub magnetic_declination_accuracy_deg: f32,
}
