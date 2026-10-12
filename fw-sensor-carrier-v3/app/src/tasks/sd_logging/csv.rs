use core::fmt::Write as _;

use embassy_time::Instant;
use heapless::String;

use crate::types::{
    BaroReading, DhtReading, GnssReading, ImuReading, MagReading, Mark, Reading, SefLogSample,
};

type Row = String<256>;

pub enum Record {
    State(SefLogSample),
    Imu(ImuReading),
    Mag(MagReading),
    Gnss(GnssReading),
    Baro(BaroReading),
    Dht(DhtReading),
    Mark(Mark),
}

pub const RECORD_KINDS: usize = 7;

pub struct CsvFile {
    pub name: &'static str,
    pub header: &'static str,
}

pub const FILE_COUNT: usize = RECORD_KINDS + 1;
pub const DROP_FILE: usize = RECORD_KINDS;
// File order matches Record, with drop counters last.
pub const FILES: [CsvFile; FILE_COUNT] = [
    CsvFile {
        name: "STATE.CSV",
        header: "cal_us,imu,selected,msl_ready,redundancy_ready,mag_ignored,height_msl_m,velocity_mps,bias0_m,bias1_m,height_std_m,velocity_std_mps,bias0_std_m,bias1_std_m,score,qw,qx,qy,qz\n",
    },
    CsvFile {
        name: "IMU.CSV",
        header: "read_us,raw_us,cal_us,imu,ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n",
    },
    CsvFile {
        name: "MAG.CSV",
        header: "read_us,raw_us,cal_us,mag,x_raw,y_raw,z_raw,x_nt,y_nt,z_nt\n",
    },
    CsvFile {
        name: "GNSS.CSV",
        header: "read_us,raw_us,cal_us,gnss,itow_ms,num_satellites,fix_type,fix_ok,latitude_deg,longitude_deg,height_msl_m,velocity_down_mps,horizontal_accuracy_mm,vertical_accuracy_mm,speed_accuracy_mps,pdop_centi\n",
    },
    CsvFile {
        name: "BARO.CSV",
        header: "read_us,raw_us,cal_us,baro,pressure_mbar,temperature_c\n",
    },
    CsvFile {
        name: "DHT.CSV",
        header: "read_us,raw_us,cal_us,dht,temperature_c,humidity_rh\n",
    },
    CsvFile {
        name: "MARKS.CSV",
        header: "uptime_us,label\n",
    },
    CsvFile {
        name: "DROPS.CSV",
        header: "uptime_us,state,imu,mag,gnss,baro,dht,mark\n",
    },
];

pub fn format_record(record: &Record) -> Result<Row, core::fmt::Error> {
    let mut row = Row::new();
    match record {
        Record::State(s) => {
            let [qw, qx, qy, qz] = s.orientation_body_to_ned_wxyz;
            writeln!(
                row,
                "{},{},{},{},{},{},{:.3},{:.4},{:.3},{:.3},{:.3},{:.4},{:.3},{:.3},{:.3},{:.6},{:.6},{:.6},{:.6}",
                s.ts.as_micros(),
                s.imu.index(),
                u8::from(s.selected),
                u8::from(s.msl_ready),
                u8::from(s.redundancy_ready),
                u8::from(s.magnetometer_ignored),
                s.height_msl_m,
                s.velocity_mps,
                s.barometer_bias_m[0],
                s.barometer_bias_m[1],
                s.height_std_m,
                s.velocity_std_mps,
                s.barometer_bias_std_m[0],
                s.barometer_bias_std_m[1],
                s.consistency_score,
                qw,
                qx,
                qy,
                qz,
            )?;
        }
        Record::Imu(Reading { raw, cal }) => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            writeln!(
                row,
                ",{},{},{},{},{},{},{:.6},{:.6},{:.6},{:.4},{:.4},{:.4}",
                raw.accel.x,
                raw.accel.y,
                raw.accel.z,
                raw.gyro.x,
                raw.gyro.y,
                raw.gyro.z,
                cal.accel.x,
                cal.accel.y,
                cal.accel.z,
                cal.gyro.x,
                cal.gyro.y,
                cal.gyro.z,
            )?;
        }
        Record::Mag(Reading { raw, cal }) => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            writeln!(
                row,
                ",{},{},{},{:.2},{:.2},{:.2}",
                raw.x, raw.y, raw.z, cal.x, cal.y, cal.z,
            )?;
        }
        Record::Gnss(Reading { raw, cal }) => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            let p = cal.pvt;
            writeln!(
                row,
                ",{},{},{},{},{:.8},{:.8},{:.3},{:.4},{},{},{:.4},{}",
                p.itow_ms,
                p.num_satellites,
                p.fix_type as u8,
                u8::from(p.fix_ok),
                p.latitude_deg,
                p.longitude_deg,
                p.height_msl_m,
                p.velocity_down_mps,
                p.horizontal_accuracy_mm,
                p.vertical_accuracy_mm,
                p.speed_accuracy_mps,
                p.pdop_centi,
            )?;
        }
        Record::Baro(Reading { raw, cal }) => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            writeln!(row, ",{:.3},{:.3}", cal.pressure_mbar, cal.temperature_c)?;
        }
        Record::Dht(Reading { raw, cal }) => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            writeln!(row, ",{:.2},{:.2}", cal.temperature_c, cal.humidity_rh)?;
        }
        Record::Mark(mark) => writeln!(row, "{},{}", mark.ts.as_micros(), mark.label)?,
    }
    Ok(row)
}

pub fn format_drops(timestamp: Instant, dropped: &[u64; RECORD_KINDS]) -> Row {
    let mut row = Row::new();
    write!(row, "{}", timestamp.as_micros()).unwrap();
    for count in dropped {
        write!(row, ",{}", count).unwrap();
    }
    row.push('\n').unwrap();
    row
}

fn write_times(
    row: &mut Row,
    read_ts: Instant,
    raw_ts: Instant,
    cal_ts: Instant,
    index: usize,
) -> core::fmt::Result {
    write!(
        row,
        "{},{},{},{}",
        read_ts.as_micros(),
        raw_ts.as_micros(),
        cal_ts.as_micros(),
        index
    )
}
