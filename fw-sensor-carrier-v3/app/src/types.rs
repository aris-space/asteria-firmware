#![allow(dead_code)]

//! Materialised per sensor samples. A sample is a measurement
//! that tracks provenance and its timestamp. Downstream code should not
//! see uncalibrated values or untimed observations.

use embassy_time::Instant;
use lsm6dso32::types::{Acceleration, AccelerationRaw, AngularRate, AngularRateRaw};

use crate::sensors::{BarometerId, DhtId, GnssId, ImuId, MagnetometerId};

#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub src: ImuId,
    pub ts: Instant,
    pub accel: Acceleration,
    pub gyro: AngularRate,
}

/// Raw IMU sample, sensor frame, native LSM6DSO32 counts (i16 LSB). Built by
/// the readout; conversion to g and dps happens at the calibration boundary in
/// `crate::calibration::imu::apply_calibration`. Never crosses a channel.
#[derive(Clone, Copy, Debug)]
pub struct RawImuSample {
    pub src: ImuId,
    pub ts: Instant,
    pub accel: AccelerationRaw,
    pub gyro: AngularRateRaw,
}

/// Pressure and temperature in sensor units, timestamped near the pressure
/// conversion by the barometer readout.
#[derive(Clone, Copy, Debug)]
pub struct BaroSample {
    pub src: BarometerId,
    pub ts: Instant,
    pub pressure_mbar: f32,
    pub temperature_c: f32,
}

/// Humidity and temperature from the readout, timestamped on completion.
#[derive(Clone, Copy, Debug)]
pub struct DhtSample {
    pub src: DhtId,
    pub ts: Instant,
    pub temperature_c: f32,
    pub humidity_rh: f32,
}

/// GNSS navigation data, timestamped when its packet is received.
#[derive(Clone, Copy, Debug)]
pub struct GnssSample {
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
    /// GPS time of week of the navigation epoch.
    pub itow_ms: u32,
    pub num_satellites: u8,
    pub fix_type: ublox::GpsFix,
    pub fix_ok: bool,
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub height_msl: f32,
    pub vel_down: f32,
    pub pdop: u16,
    pub horiz_accuracy: u32,
    pub vert_accuracy: u32,
    pub speed_accuracy_mps: f32,
}

/// Selected SEF-light state, including MSL height and body-to-NED attitude.
#[derive(Clone, Copy, Debug)]
pub struct StateEstimate {
    pub ts: Instant,
    /// Height and vertical velocity may be sent as MSL telemetry once true.
    pub msl_ready: bool,
    pub height_msl_m: f32,
    pub velocity_mps: f32,
    pub height_std_m: f32,
    pub velocity_std_mps: f32,
    pub orientation_body_to_ned_wxyz: [f32; 4],
    pub selected_imu: ImuId,
    pub redundancy_ready: bool,
}

/// One SEF chain's state at the output cadence, before selecting an IMU.
#[derive(Clone, Copy, Debug)]
pub struct SefLogSample {
    pub ts: Instant,
    pub imu: ImuId,
    pub selected: bool,
    pub msl_ready: bool,
    pub redundancy_ready: bool,
    pub selected_gnss: Option<GnssId>,
    pub height_msl_m: f32,
    pub velocity_mps: f32,
    pub barometer_bias_m: [f32; 2],
    pub height_std_m: f32,
    pub velocity_std_mps: f32,
    pub barometer_bias_std_m: [f32; 2],
    pub consistency_score: f32,
    pub orientation_body_to_ned_wxyz: [f32; 4],
}

/// One row of the SD card CSV logs. Sensor rows keep the value before and
/// after calibration and the instant the readout received it, so latency and
/// calibration can be refitted offline.
#[derive(Clone, Copy, Debug)]
pub enum SdLogRecord {
    State(SefLogSample),
    Imu {
        raw: RawImuSample,
        cal: ImuSample,
        read_ts: Instant,
    },
    Magnetometer {
        raw: RawMagSample,
        cal: MagSample,
    },
    Gnss(GnssSample),
    Barometer {
        sample: BaroSample,
        read_ts: Instant,
    },
    Dht(DhtSample),
}

impl SdLogRecord {
    pub const KIND_COUNT: usize = 6;

    /// Selects this record's CSV file and drop counter.
    pub const fn kind(&self) -> usize {
        match self {
            Self::State(_) => 0,
            Self::Imu { .. } => 1,
            Self::Magnetometer { .. } => 2,
            Self::Gnss(_) => 3,
            Self::Barometer { .. } => 4,
            Self::Dht(_) => 5,
        }
    }
}
