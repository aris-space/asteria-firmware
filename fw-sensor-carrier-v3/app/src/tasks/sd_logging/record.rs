//! Log records as COBS-framed postcard values. A zero byte ends every frame,
//! so a reader that lost data resyncs at the next one.

use embassy_time::Instant;
use postcard_schema::Schema;
use serde::Serialize;

use crate::types::{
    BaroReading, DhtReading, GnssReading, ImuReading, MagReading, Mark, Reading, SefLogSample,
};

/// One log record. Each variant is a table and its fields are the columns, in
/// order. `schema_json` describes this enum in the log directory.
#[derive(Serialize, Schema)]
pub enum Record {
    State {
        cal_us: u64,
        imu: u8,
        selected: bool,
        msl_ready: bool,
        redundancy_ready: bool,
        mag_ignored: bool,
        height_msl_m: f32,
        velocity_mps: f32,
        bias0_m: f32,
        bias1_m: f32,
        height_std_m: f32,
        velocity_std_mps: f32,
        bias0_std_m: f32,
        bias1_std_m: f32,
        score: f32,
        qw: f32,
        qx: f32,
        qy: f32,
        qz: f32,
    },
    Imu {
        read_us: u64,
        raw_us: u64,
        cal_us: u64,
        imu: u8,
        ax_raw: i16,
        ay_raw: i16,
        az_raw: i16,
        gx_raw: i16,
        gy_raw: i16,
        gz_raw: i16,
        ax_g: f32,
        ay_g: f32,
        az_g: f32,
        gx_dps: f32,
        gy_dps: f32,
        gz_dps: f32,
    },
    Mag {
        read_us: u64,
        raw_us: u64,
        cal_us: u64,
        mag: u8,
        x_raw: i16,
        y_raw: i16,
        z_raw: i16,
        x_nt: f32,
        y_nt: f32,
        z_nt: f32,
    },
    Gnss {
        read_us: u64,
        raw_us: u64,
        cal_us: u64,
        gnss: u8,
        itow_ms: u32,
        num_satellites: u8,
        fix_type: u8,
        fix_ok: bool,
        latitude_deg: f64,
        longitude_deg: f64,
        height_msl_m: f32,
        velocity_down_mps: f32,
        horizontal_accuracy_mm: u32,
        vertical_accuracy_mm: u32,
        speed_accuracy_mps: f32,
        pdop_centi: u16,
    },
    Baro {
        read_us: u64,
        raw_us: u64,
        cal_us: u64,
        baro: u8,
        pressure_mbar: f32,
        temperature_c: f32,
    },
    Dht {
        read_us: u64,
        raw_us: u64,
        cal_us: u64,
        dht: u8,
        temperature_c: f32,
        humidity_rh: f32,
    },
    Marks {
        uptime_us: u64,
        label: heapless::String<{ Mark::LABEL_LEN }>,
    },
    /// Readings lost per table since the previous drops record.
    Drops {
        uptime_us: u64,
        state: u64,
        imu: u64,
        mag: u64,
        gnss: u64,
        baro: u64,
        dht: u64,
        marks: u64,
    },
}

/// Tables that have a drop counter, in drop counter order.
#[derive(Clone, Copy)]
pub enum Kind {
    State,
    Imu,
    Mag,
    Gnss,
    Baro,
    Dht,
    Marks,
}

pub const RECORD_KINDS: usize = Kind::Marks as usize + 1;
pub const MAX_FRAME: usize = 128;

impl Record {
    pub fn drops(uptime: Instant, dropped: &[u64; RECORD_KINDS]) -> Self {
        Record::Drops {
            uptime_us: uptime.as_micros(),
            state: dropped[Kind::State as usize],
            imu: dropped[Kind::Imu as usize],
            mag: dropped[Kind::Mag as usize],
            gnss: dropped[Kind::Gnss as usize],
            baro: dropped[Kind::Baro as usize],
            dht: dropped[Kind::Dht as usize],
            marks: dropped[Kind::Marks as usize],
        }
    }

    pub fn encode<'a>(&self, buf: &'a mut [u8; MAX_FRAME]) -> &'a [u8] {
        postcard::to_slice_cobs(self, buf).expect("record fits MAX_FRAME")
    }
}

impl From<SefLogSample> for Record {
    fn from(s: SefLogSample) -> Self {
        let [qw, qx, qy, qz] = s.orientation_body_to_ned_wxyz;
        Record::State {
            cal_us: s.ts.as_micros(),
            imu: s.imu.index() as u8,
            selected: s.selected,
            msl_ready: s.msl_ready,
            redundancy_ready: s.redundancy_ready,
            mag_ignored: s.magnetometer_ignored,
            height_msl_m: s.height_msl_m,
            velocity_mps: s.velocity_mps,
            bias0_m: s.barometer_bias_m[0],
            bias1_m: s.barometer_bias_m[1],
            height_std_m: s.height_std_m,
            velocity_std_mps: s.velocity_std_mps,
            bias0_std_m: s.barometer_bias_std_m[0],
            bias1_std_m: s.barometer_bias_std_m[1],
            score: s.consistency_score,
            qw,
            qx,
            qy,
            qz,
        }
    }
}

impl From<ImuReading> for Record {
    fn from(Reading { raw, cal }: ImuReading) -> Self {
        Record::Imu {
            read_us: raw.read_ts.as_micros(),
            raw_us: raw.ts.as_micros(),
            cal_us: cal.ts.as_micros(),
            imu: raw.src.index() as u8,
            ax_raw: raw.accel.x,
            ay_raw: raw.accel.y,
            az_raw: raw.accel.z,
            gx_raw: raw.gyro.x,
            gy_raw: raw.gyro.y,
            gz_raw: raw.gyro.z,
            ax_g: cal.accel.x,
            ay_g: cal.accel.y,
            az_g: cal.accel.z,
            gx_dps: cal.gyro.x,
            gy_dps: cal.gyro.y,
            gz_dps: cal.gyro.z,
        }
    }
}

impl From<MagReading> for Record {
    fn from(Reading { raw, cal }: MagReading) -> Self {
        Record::Mag {
            read_us: raw.read_ts.as_micros(),
            raw_us: raw.ts.as_micros(),
            cal_us: cal.ts.as_micros(),
            mag: raw.src.index() as u8,
            x_raw: raw.x,
            y_raw: raw.y,
            z_raw: raw.z,
            x_nt: cal.x,
            y_nt: cal.y,
            z_nt: cal.z,
        }
    }
}

impl From<GnssReading> for Record {
    fn from(Reading { raw, cal }: GnssReading) -> Self {
        let p = cal.pvt;
        Record::Gnss {
            read_us: raw.read_ts.as_micros(),
            raw_us: raw.ts.as_micros(),
            cal_us: cal.ts.as_micros(),
            gnss: raw.src.index() as u8,
            itow_ms: p.itow_ms,
            num_satellites: p.num_satellites,
            fix_type: p.fix_type as u8,
            fix_ok: p.fix_ok,
            latitude_deg: p.latitude_deg,
            longitude_deg: p.longitude_deg,
            height_msl_m: p.height_msl_m,
            velocity_down_mps: p.velocity_down_mps,
            horizontal_accuracy_mm: p.horizontal_accuracy_mm,
            vertical_accuracy_mm: p.vertical_accuracy_mm,
            speed_accuracy_mps: p.speed_accuracy_mps,
            pdop_centi: p.pdop_centi,
        }
    }
}

impl From<BaroReading> for Record {
    fn from(Reading { raw, cal }: BaroReading) -> Self {
        Record::Baro {
            read_us: raw.read_ts.as_micros(),
            raw_us: raw.ts.as_micros(),
            cal_us: cal.ts.as_micros(),
            baro: raw.src.index() as u8,
            pressure_mbar: cal.pressure_mbar,
            temperature_c: cal.temperature_c,
        }
    }
}

impl From<DhtReading> for Record {
    fn from(Reading { raw, cal }: DhtReading) -> Self {
        Record::Dht {
            read_us: raw.read_ts.as_micros(),
            raw_us: raw.ts.as_micros(),
            cal_us: cal.ts.as_micros(),
            dht: raw.src.index() as u8,
            temperature_c: cal.temperature_c,
            humidity_rh: cal.humidity_rh,
        }
    }
}

impl From<Mark> for Record {
    fn from(mark: Mark) -> Self {
        Record::Marks {
            uptime_us: mark.ts.as_micros(),
            label: mark.label,
        }
    }
}

/// Serialises postcard-schema's description of `Record` as JSON, for the
/// `SCHEMA.TXT` that goes next to the log.
pub fn schema_json(buf: &mut [u8]) -> &[u8] {
    let len = serde_json_core::to_slice(Record::SCHEMA, buf).expect("schema fits its buffer");
    &buf[..len]
}
