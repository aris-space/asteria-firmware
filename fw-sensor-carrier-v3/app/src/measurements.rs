#![allow(dead_code)]

use embassy_time::{Duration, Instant};
use lsm6dso32::types::{Acceleration, AngularRate};

use crate::sensors::{BarometerId, GnssId, ImuId, MagnetometerId};

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

/// Snapshot of the EKF after an IMU prediction step.
///
/// Numeric fields are kept as plain `f64` arrays so this type does not depend
/// on which `nalgebra` version any particular consumer is built against (the
/// firmware uses 0.33, the `ekf` crate uses 0.34).
#[derive(Clone, Copy, Debug)]
pub struct StateEstimate {
    pub ts: Instant,
    /// NED position [N, E, D] in meters.
    pub pos_ned_m: [f64; 3],
    /// NED velocity [VN, VE, VD] in m/s.
    pub vel_ned_mps: [f64; 3],
    /// Body-to-NED attitude quaternion [w, x, y, z].
    pub attitude_quat_wxyz: [f64; 4],
    /// Diagonal of the 9x9 covariance (variance of each state).
    pub cov_diag: [f64; 9],
}

#[derive(Clone, Copy, Debug)]
pub struct PvtData {
    pub lon_deg: f64,
    pub lat_deg: f64,
    pub fix_type: ublox::GpsFix,
    /// Height above WGS84 ellipsoid, in metres. Required by the EKF's
    /// `Gnss::from_geodetic` (`map_3d::geodetic2ned` expects ellipsoidal alt).
    pub height_ellipsoid_m: f32,
    /// Height above mean sea level, in metres. Kept for human-facing telemetry;
    /// not used by the EKF.
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
