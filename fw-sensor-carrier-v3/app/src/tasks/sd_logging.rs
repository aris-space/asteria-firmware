//! Full-rate CSV logging through the async SDMMC and FAT drivers.
//!
//! Each session writes one CSV per record kind into a new `LOGnnnn` directory.
//! Sensor files start with the same columns:
//!
//! - `read_us`: when the readout received the data,
//! - `raw_us`: the readout's estimate of the measurement time; IMU samples are
//!   interpolated across their FIFO batch, barometer samples sit at the middle
//!   of the conversion, other sensors use `read_us`,
//! - `cal_us`: `raw_us` minus the stored latency, the time the estimator uses,
//! - the sensor index.
//!
//! IMU and magnetometer rows then hold raw sensor-frame counts (accel 4096
//! LSB/g, gyro 70 mdps/LSB, mag 150 nT/LSB) followed by calibrated board-frame
//! values. Other sensors have no value correction yet, so their values appear
//! once.

use core::fmt::Write as _;

use block_device_adapters::BufStream;
use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, Either6, select, select6};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::sdmmc::sd::{Addressable, CmdBlock, StorageDevice};
use embassy_stm32::time::mhz;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_fatfs::{Error as FatError, FileSystem, FsOptions};
use embedded_io_async::Write;
use embedded_partitions::mbr::Mbr;
use heapless::{String, Vec};

use crate::resources::sd::Sd;
use crate::signals;
use crate::types::{
    BaroReading, DhtReading, GnssReading, ImuReading, MagReading, Reading, SefLogSample,
};

struct CsvFile {
    name: &'static str,
    header: &'static str,
}

const FILE_COUNT: usize = RECORD_KINDS + 1;
const DROP_FILE: usize = RECORD_KINDS;
// One file per `Record` kind, in its order, followed by the drop counters.
const FILES: [CsvFile; FILE_COUNT] = [
    CsvFile {
        name: "STATE.CSV",
        header: "cal_us,imu,selected,msl_ready,redundancy_ready,height_msl_m,velocity_mps,bias0_m,bias1_m,height_std_m,velocity_std_mps,bias0_std_m,bias1_std_m,score,qw,qx,qy,qz\n",
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
        name: "DROPS.CSV",
        header: "uptime_us,state,imu,mag,gnss,baro,dht\n",
    },
];
const BUFFER_SIZE: usize = 4096;
const FLUSH_PERIOD: Duration = Duration::from_secs(1);

type Row = String<256>;

/// A CSV file with a RAM buffer, so the card sees few large writes.
struct CsvLog<F> {
    name: &'static str,
    file: F,
    buffer: [u8; BUFFER_SIZE],
    used: usize,
    rows: u32,
}

impl<F: Write> CsvLog<F> {
    fn new(spec: &CsvFile, file: F) -> Self {
        let mut log = Self {
            name: spec.name,
            file,
            buffer: [0; BUFFER_SIZE],
            used: spec.header.len(),
            rows: 0,
        };
        log.buffer[..log.used].copy_from_slice(spec.header.as_bytes());
        log
    }

    async fn append(&mut self, row: &[u8]) -> Result<(), F::Error> {
        if self.used + row.len() > BUFFER_SIZE {
            self.write_buffer().await?;
        }
        self.buffer[self.used..][..row.len()].copy_from_slice(row);
        self.used += row.len();
        self.rows += 1;
        Ok(())
    }

    async fn write_buffer(&mut self) -> Result<(), F::Error> {
        self.file.write_all(&self.buffer[..self.used]).await?;
        self.used = 0;
        Ok(())
    }

    /// Commits buffered rows to the card and returns how many rows that was.
    async fn flush(&mut self) -> Result<u32, F::Error> {
        self.write_buffer().await?;
        self.file.flush().await?;
        Ok(core::mem::take(&mut self.rows))
    }
}

#[embassy_executor::task]
pub async fn task(mut sdmmc: Sd, detect: Input<'static>, _power: Output<'static>) {
    info!(
        "SD: card detect pin is {}",
        if detect.is_high() { "high" } else { "low" }
    );
    // Allow the switched SD supply to settle after setup.
    Timer::after(Duration::from_millis(250)).await;
    loop {
        let _ = run_session(&mut sdmmc).await;
        Timer::after(Duration::from_secs(5)).await;
    }
}

/// Mounts the card and logs to a new directory until a card error ends the
/// session. Every error has been reported by the time this returns.
async fn run_session(sdmmc: &mut Sd) -> Result<(), ()> {
    let mut cmd_block = CmdBlock::new();
    let card = with_timeout(
        Duration::from_secs(2),
        StorageDevice::new_sd_card(sdmmc, &mut cmd_block, mhz(25)),
    )
    .await
    .map_err(|_| warn!("SD: card init timed out"))?
    .map_err(|e| warn!("SD: card init failed: {}", Debug2Format(&e)))?;
    info!(
        "SD: initialized ({} MiB)",
        card.card().size() / (1024 * 1024)
    );

    let mbr = Mbr::new(BufStream::<_, 512>::new(card))
        .await
        .map_err(|e| warn!("SD: MBR read failed: {}", Debug2Format(&e)))?;
    let (partition, _) = mbr
        .iter_used()
        .find(|(_, part)| part.is_fat())
        .ok_or_else(|| warn!("SD: no FAT partition found"))?;
    let slice = mbr
        .into_partition(partition)
        .await
        .map_err(|e| warn!("SD: partition open failed: {}", Debug2Format(&e)))?;
    let fs = FileSystem::new(slice, FsOptions::new())
        .await
        .map_err(|e| warn!("SD: FAT mount failed: {}", Debug2Format(&e)))?;

    let root = fs.root_dir();
    let mut dir_name = String::<8>::new();
    let mut found = false;
    for number in 1..=9999 {
        dir_name.clear();
        write!(dir_name, "LOG{:04}", number).unwrap();
        match root.open_dir(&dir_name).await {
            Ok(_) => continue,
            Err(FatError::NotFound) => {
                found = true;
                break;
            }
            Err(e) => {
                warn!("SD: log directory lookup failed: {}", Debug2Format(&e));
                return Err(());
            }
        }
    }
    if !found {
        warn!("SD: all log directory names are occupied");
        return Err(());
    }
    let dir = root.create_dir(&dir_name).await.map_err(|e| {
        warn!(
            "SD: {} create failed: {}",
            dir_name.as_str(),
            Debug2Format(&e)
        )
    })?;

    let mut logs = Vec::<_, FILE_COUNT>::new();
    for spec in &FILES {
        let file = dir
            .create_file(spec.name)
            .await
            .map_err(|e| warn!("SD: {} create failed: {}", spec.name, Debug2Format(&e)))?;
        let _ = logs.push(CsvLog::new(spec, file));
    }
    info!("SD: logging to {}", dir_name.as_str());

    let mut state = signals::STATE_CHANNEL
        .subscriber()
        .expect("SD: subscriber slot");
    let mut imu = signals::IMU_CHANNEL
        .subscriber()
        .expect("SD: subscriber slot");
    let mut mag = signals::MAG_CHANNEL
        .subscriber()
        .expect("SD: subscriber slot");
    let mut gnss = signals::GNSS_CHANNEL
        .subscriber()
        .expect("SD: subscriber slot");
    let mut baro = signals::BARO_CHANNEL
        .subscriber()
        .expect("SD: subscriber slot");
    let mut dht = signals::DHT_CHANNEL
        .subscriber()
        .expect("SD: subscriber slot");
    let mut dropped = [0u64; RECORD_KINDS];
    let mut last_flush = Instant::now();
    loop {
        let next = select6(
            state.next_message(),
            imu.next_message(),
            mag.next_message(),
            gnss.next_message(),
            baro.next_message(),
            dht.next_message(),
        );
        if let Either::First(next) = select(next, Timer::at(last_flush + FLUSH_PERIOD)).await {
            let (kind, received) = match next {
                Either6::First(message) => (0, received(message, Record::State)),
                Either6::Second(message) => (1, received(message, Record::Imu)),
                Either6::Third(message) => (2, received(message, Record::Mag)),
                Either6::Fourth(message) => (3, received(message, Record::Gnss)),
                Either6::Fifth(message) => (4, received(message, Record::Baro)),
                Either6::Sixth(message) => (5, received(message, Record::Dht)),
            };
            match received {
                Ok(record) => {
                    let log = &mut logs[kind];
                    let Ok(row) = format_record(&record) else {
                        warn!("SD: {} row exceeded buffer", log.name);
                        continue;
                    };
                    log.append(row.as_bytes()).await.map_err(|e| {
                        warn!("SD: {} write failed: {}", log.name, Debug2Format(&e))
                    })?;
                }
                Err(lost) => dropped[kind] += lost,
            }
        }
        if Instant::now() < last_flush + FLUSH_PERIOD {
            continue;
        }

        if dropped.iter().any(|&count| count > 0) {
            let mut row = Row::new();
            write!(row, "{}", Instant::now().as_micros()).unwrap();
            for count in dropped {
                write!(row, ",{}", count).unwrap();
            }
            row.push('\n').unwrap();
            let log = &mut logs[DROP_FILE];
            log.append(row.as_bytes())
                .await
                .map_err(|e| warn!("SD: {} write failed: {}", log.name, Debug2Format(&e)))?;
        }

        let mut rows = [0u32; FILE_COUNT];
        for (log, rows) in logs.iter_mut().zip(&mut rows) {
            *rows = log
                .flush()
                .await
                .map_err(|e| warn!("SD: {} flush failed: {}", log.name, Debug2Format(&e)))?;
        }
        info!(
            "SD: flushed state={}, IMU={}, mag={}, GNSS={}, baro={}, DHT={}, dropped={}",
            rows[0], rows[1], rows[2], rows[3], rows[4], rows[5], dropped,
        );
        dropped = [0; RECORD_KINDS];
        last_flush = Instant::now();
    }
}

/// One row for the CSV file of the same index in [`FILES`].
enum Record {
    State(SefLogSample),
    Imu(ImuReading),
    Mag(MagReading),
    Gnss(GnssReading),
    Baro(BaroReading),
    Dht(DhtReading),
}

const RECORD_KINDS: usize = 6;

fn received<T: Clone>(message: WaitResult<T>, record: fn(T) -> Record) -> Result<Record, u64> {
    match message {
        WaitResult::Message(value) => Ok(record(value)),
        WaitResult::Lagged(lost) => Err(lost),
    }
}

fn format_record(record: &Record) -> Result<Row, core::fmt::Error> {
    let mut row = Row::new();
    match record {
        Record::State(s) => {
            let [qw, qx, qy, qz] = s.orientation_body_to_ned_wxyz;
            writeln!(
                row,
                "{},{},{},{},{},{:.3},{:.4},{:.3},{:.3},{:.3},{:.4},{:.3},{:.3},{:.3},{:.6},{:.6},{:.6},{:.6}",
                s.ts.as_micros(),
                s.imu.index(),
                u8::from(s.selected),
                u8::from(s.msl_ready),
                u8::from(s.redundancy_ready),
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
    }
    Ok(row)
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
