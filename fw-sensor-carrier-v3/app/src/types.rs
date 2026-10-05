#![allow(dead_code)]

//! Per-sensor samples and the records derived from them. A sample is a
//! measurement with its source and timestamp. Only readouts and the SD log see
//! raw samples; everything downstream uses calibrated ones.

use embassy_time::Instant;
use lsm6dso32::types::{Acceleration, AccelerationRaw, AngularRate, AngularRateRaw};

use crate::sensors::{BaroId, DhtId, GnssId, ImuId, MagId};

// Every sensor has a raw sample, built by its readout, and a calibrated
// sample, built by `crate::calibration::<kind>::apply_calibration`. A raw `ts`
// is the readout's estimate of the measurement time; `read_ts` is when the
// readout received the data. A calibrated `ts` also removes the stored latency.

/// Raw IMU sample, sensor frame, native LSM6DSO32 counts (i16 LSB).
#[derive(Clone, Copy, Debug)]
pub struct RawImuSample {
    pub src: ImuId,
    pub ts: Instant,
    pub read_ts: Instant,
    pub accel: AccelerationRaw,
    pub gyro: AngularRateRaw,
}

/// Calibrated IMU sample, board frame, g and deg/s.
#[derive(Clone, Copy, Debug)]
pub struct ImuSample {
    pub src: ImuId,
    pub ts: Instant,
    pub accel: Acceleration,
    pub gyro: AngularRate,
}

/// Raw magnetometer sample, sensor frame, native LSM303AGR counts (i16 LSB).
/// The cal solver fits these counts directly.
#[derive(Clone, Copy, Debug)]
pub struct RawMagSample {
    pub src: MagId,
    pub ts: Instant,
    pub read_ts: Instant,
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Calibrated magnetometer sample, board frame, nT.
#[derive(Clone, Copy, Debug)]
pub struct MagSample {
    pub src: MagId,
    pub ts: Instant,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Raw GNSS navigation data, stamped when its packet arrived.
#[derive(Clone, Copy, Debug)]
pub struct RawGnssSample {
    pub src: GnssId,
    pub ts: Instant,
    pub read_ts: Instant,
    pub pvt: Pvt,
}

#[derive(Clone, Copy, Debug)]
pub struct GnssSample {
    pub src: GnssId,
    pub ts: Instant,
    pub pvt: Pvt,
}

/// Raw barometer sample, factory-compensated by the MS5607, stamped at the
/// middle of the pressure conversion.
#[derive(Clone, Copy, Debug)]
pub struct RawBaroSample {
    pub src: BaroId,
    pub ts: Instant,
    pub read_ts: Instant,
    pub pressure_mbar: f32,
    pub temperature_c: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct BaroSample {
    pub src: BaroId,
    pub ts: Instant,
    pub pressure_mbar: f32,
    pub temperature_c: f32,
}

impl BaroSample {
    /// ISA pressure altitude. It differs from MSL height by the local
    /// sea-level pressure, which the estimator's bias states absorb.
    pub fn pressure_altitude_m(&self) -> f32 {
        44_330.0 * (1.0 - libm::powf(self.pressure_mbar / 1_013.25, 0.190_294_95))
    }
}

/// Raw humidity and temperature sample, stamped on completion.
#[derive(Clone, Copy, Debug)]
pub struct RawDhtSample {
    pub src: DhtId,
    pub ts: Instant,
    pub read_ts: Instant,
    pub temperature_c: f32,
    pub humidity_rh: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct DhtSample {
    pub src: DhtId,
    pub ts: Instant,
    pub temperature_c: f32,
    pub humidity_rh: f32,
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
    pub height_msl_m: f32,
    pub velocity_down_mps: f32,
    pub pdop_centi: u16,
    pub horizontal_accuracy_mm: u32,
    pub vertical_accuracy_mm: u32,
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
    pub height_msl_m: f32,
    pub velocity_mps: f32,
    pub barometer_bias_m: [f32; 2],
    pub height_std_m: f32,
    pub velocity_std_mps: f32,
    pub barometer_bias_std_m: [f32; 2],
    pub consistency_score: f32,
    pub orientation_body_to_ned_wxyz: [f32; 4],
}

/// One row of the SD card CSV logs. Sensor rows keep the sample before and
/// after calibration, so latency and calibration can be refitted offline.
#[derive(Clone, Copy, Debug)]
pub enum SdLogRecord {
    State(SefLogSample),
    Imu { raw: RawImuSample, cal: ImuSample },
    Mag { raw: RawMagSample, cal: MagSample },
    Gnss { raw: RawGnssSample, cal: GnssSample },
    Baro { raw: RawBaroSample, cal: BaroSample },
    Dht { raw: RawDhtSample, cal: DhtSample },
}

impl SdLogRecord {
    pub const KIND_COUNT: usize = 6;

    /// Selects this record's CSV file and drop counter.
    pub const fn kind(&self) -> usize {
        match self {
            Self::State(_) => 0,
            Self::Imu { .. } => 1,
            Self::Mag { .. } => 2,
            Self::Gnss { .. } => 3,
            Self::Baro { .. } => 4,
            Self::Dht { .. } => 5,
        }
    }
}
