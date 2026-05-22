#![allow(dead_code)]

use embassy_time::Instant;
use lsm6dso32::types::{Acceleration, AngularRate};
use nalgebra::UnitQuaternion;

use crate::sensors::{BarometerId, DhtId, GnssId, ImuId, MagnetometerId};

// Per-sensor samples. Each carries its source sensor id and a measurement
// timestamp inline; downstream code never sees uncalibrated values or
// untimed observations.

#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
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

#[derive(Clone, Copy, Debug)]
pub struct DhtSample {
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

/// Raw magnetometer sample, sensor frame, raw counts.
/// Only crosses the readout-task boundary; calibration in the processing
/// task converts this to `MagSample`.
#[derive(Clone, Copy, Debug)]
pub struct RawMagSample {
    pub src: MagnetometerId,
    pub ts: Instant,
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Calibrated magnetometer sample, board frame, nT.
/// Output of the magnetic-field processing task; consumed by inertial
/// fusion and the CAN layer.
#[derive(Clone, Copy, Debug)]
pub struct MagSample {
    pub src: MagnetometerId,
    pub ts: Instant,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// GNSS PVT payload. Kept as a substruct of `GnssSample` because flattening
/// 16 fields would bury the metadata.
#[derive(Clone, Copy, Debug)]
pub struct Pvt {
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

// Fused / derived signals. Board-internal types; conversion to CAN wire
// formats lives in the CAN tx layer. Every type carries `ts`, propagated
// from the input sample(s) that drove the update.

#[derive(Clone, Copy, Debug)]
pub struct Pressure {
    pub ts: Instant,
    pub mbar: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Orientation {
    pub ts: Instant,
    pub q: UnitQuaternion<f32>,
}

#[derive(Clone, Copy, Debug)]
pub struct Environment {
    pub ts: Instant,
    pub temperature_c: f32,
    pub humidity_rh: f32,
    pub pressure_mbar: f32,
}

/// Body-frame accel/gyro plus NED-rotated, gravity-compensated accel/gyro.
#[derive(Clone, Copy, Debug)]
pub struct Inertial {
    pub ts: Instant,
    pub body_accel_x: f32,
    pub body_accel_y: f32,
    pub body_accel_z: f32,
    pub body_gyro_x: f32,
    pub body_gyro_y: f32,
    pub body_gyro_z: f32,
    pub ned_accel_north: f32,
    pub ned_accel_east: f32,
    pub ned_accel_down: f32,
    pub ned_gyro_north: f32,
    pub ned_gyro_east: f32,
    pub ned_gyro_down: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Position {
    pub ts: Instant,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub height_msl_m: f32,
    pub horizontal_accuracy_m: f32,
    pub vertical_accuracy_m: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct Velocity {
    pub ts: Instant,
    pub body_x: f32,
    pub body_y: f32,
    pub body_z: f32,
    pub ned_north: f32,
    pub ned_east: f32,
    pub ned_down: f32,
}
