#![allow(dead_code)]

//! Materialised per sensor samples. A sample is a measurement
//! that tracks provenance and its timestamp. Downstream code should not
//! see uncalibrated values or untimed observations.

use embassy_time::Instant;
use lsm6dso32::types::{Acceleration, AngularRate};

use crate::sensors::{BarometerId, DhtId, GnssId, ImuId, MagnetometerId};

#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub src: ImuId,
    pub ts: Instant,
    pub accel: Acceleration,
    pub gyro: AngularRate,
}

/// Raw IMU sample, sensor frame. Built by the readout, consumed by
/// `crate::calibration::imu::apply_calibration`, never crosses a channel.
#[derive(Clone, Copy, Debug)]
pub struct RawImuSample {
    pub src: ImuId,
    pub ts: Instant,
    pub accel: Acceleration,
    pub gyro: AngularRate,
}

#[derive(Clone, Copy, Debug)]
pub struct BaroSample {
    pub src: BarometerId,
    pub ts: Instant,
    pub pressure_mbar: f32,
    pub temperature_c: f32,
}

/// Raw barometer sample. Built by the readout with `ts = Instant::now()`
/// (read-completion time); calibration subtracts the per-sensor delay.
#[derive(Clone, Copy, Debug)]
pub struct RawBaroSample {
    pub src: BarometerId,
    pub ts: Instant,
    pub pressure_mbar: f32,
    pub temperature_c: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct DhtSample {
    pub src: DhtId,
    pub ts: Instant,
    pub temperature_c: f32,
    pub humidity_rh: f32,
}

/// Raw DHT sample. Built by the readout with `ts = Instant::now()`
/// (read-completion time); calibration subtracts the per-sensor delay.
#[derive(Clone, Copy, Debug)]
pub struct RawDhtSample {
    pub src: DhtId,
    pub ts: Instant,
    pub temperature_c: f32,
    pub humidity_rh: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct GnssSample {
    pub src: GnssId,
    pub ts: Instant,
    pub pvt: Pvt,
}

/// Raw GNSS sample. Built by the readout with `ts = Instant::now()`
/// (read-completion time); calibration subtracts the per-sensor delay.
#[derive(Clone, Copy, Debug)]
pub struct RawGnssSample {
    pub src: GnssId,
    pub ts: Instant,
    pub pvt: Pvt,
}

/// Raw magnetometer sample, sensor frame, native LSM303AGR counts (i16 LSB).
/// The cal solver fits these counts directly; conversion to physical units (nT)
/// happens at the calibration boundary in
/// `crate::calibration::mag::apply_calibration`.
#[derive(Clone, Copy, Debug)]
pub struct RawMagSample {
    pub src: MagnetometerId,
    pub ts: Instant,
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Calibrated magnetometer sample, board frame, nT.
/// Output of magnetometer calibration.
#[derive(Clone, Copy, Debug)]
pub struct MagSample {
    pub src: MagnetometerId,
    pub ts: Instant,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// GNSS vertical position and velocity used by SEF-light.
#[derive(Clone, Copy, Debug)]
pub struct Pvt {
    pub fix_type: ublox::GpsFix,
    pub height_msl: f32,
    pub vel_down: f32,
    pub pdop: u16,
    pub vert_accuracy: u32,
    pub speed_accuracy_mps: f32,
}

/// MSL altitude and up-positive vertical velocity from SEF-light.
#[derive(Clone, Copy, Debug)]
pub struct VerticalEstimate {
    pub ts: Instant,
    pub height_msl_m: f32,
    pub velocity_mps: f32,
    pub height_std_m: f32,
    pub velocity_std_mps: f32,
    pub selected_imu: ImuId,
    pub redundancy_ready: bool,
}
