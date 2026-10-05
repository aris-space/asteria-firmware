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
use core::sync::atomic::Ordering;

use block_device_adapters::BufStream;
use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::sdmmc::sd::{Addressable, CmdBlock, StorageDevice};
use embassy_stm32::time::mhz;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_fatfs::{Error as FatError, FileSystem, FsOptions};
use embedded_io_async::Write;
use embedded_partitions::mbr::Mbr;
use heapless::{String, Vec};

use crate::resources::sd::Sd;
use crate::signals::{SD_LOG_CHANNEL, SD_LOG_DROPPED};
use crate::types::SdLogRecord;

struct CsvFile {
    name: &'static str,
    header: &'static str,
}

const FILE_COUNT: usize = SdLogRecord::KIND_COUNT + 1;
const DROP_FILE: usize = SdLogRecord::KIND_COUNT;
// Indexed by `SdLogRecord::kind`, followed by the drop counters.
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
        run_session(&mut sdmmc).await;
        Timer::after(Duration::from_secs(5)).await;
    }
}

async fn run_session(sdmmc: &mut Sd) {
    let mut cmd_block = CmdBlock::new();
    let card = match with_timeout(
        Duration::from_secs(2),
        StorageDevice::new_sd_card(sdmmc, &mut cmd_block, mhz(25)),
    )
    .await
    {
        Ok(Ok(card)) => card,
        Ok(Err(e)) => {
            warn!("SD: card init failed: {}", Debug2Format(&e));
            return;
        }
        Err(_) => {
            warn!("SD: card init timed out");
            return;
        }
    };
    info!(
        "SD: initialized ({} MiB)",
        card.card().size() / (1024 * 1024)
    );

    let mbr = match Mbr::new(BufStream::<_, 512>::new(card)).await {
        Ok(mbr) => mbr,
        Err(e) => {
            warn!("SD: MBR read failed: {}", Debug2Format(&e));
            return;
        }
    };
    let Some(partition) = mbr
        .iter_used()
        .find(|(_, part)| part.is_fat())
        .map(|(i, _)| i)
    else {
        warn!("SD: no FAT partition found");
        return;
    };
    let slice = match mbr.into_partition(partition).await {
        Ok(slice) => slice,
        Err(e) => {
            warn!("SD: partition open failed: {}", Debug2Format(&e));
            return;
        }
    };
    let fs = match FileSystem::new(slice, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(e) => {
            warn!("SD: FAT mount failed: {}", Debug2Format(&e));
            return;
        }
    };

    let root = fs.root_dir();
    let mut dir_name = String::<8>::new();
    let mut dir = None;
    for number in 1..=9999 {
        dir_name.clear();
        write!(dir_name, "LOG{:04}", number).unwrap();
        match root.open_dir(&dir_name).await {
            Ok(_) => continue,
            Err(FatError::NotFound) => {}
            Err(e) => {
                warn!("SD: log directory lookup failed: {}", Debug2Format(&e));
                return;
            }
        }
        match root.create_dir(&dir_name).await {
            Ok(created) => {
                dir = Some(created);
                break;
            }
            Err(e) => {
                warn!(
                    "SD: {} create failed: {}",
                    dir_name.as_str(),
                    Debug2Format(&e)
                );
                return;
            }
        }
    }
    let Some(dir) = dir else {
        warn!("SD: all log directory names are occupied");
        return;
    };

    let mut logs = Vec::<_, FILE_COUNT>::new();
    for spec in &FILES {
        match dir.create_file(spec.name).await {
            Ok(file) => {
                let _ = logs.push(CsvLog::new(spec, file));
            }
            Err(e) => {
                warn!("SD: {} create failed: {}", spec.name, Debug2Format(&e));
                return;
            }
        }
    }
    info!("SD: logging to {}", dir_name.as_str());

    let mut last_flush = Instant::now();
    loop {
        if let Either::First(record) = select(
            SD_LOG_CHANNEL.receive(),
            Timer::at(last_flush + FLUSH_PERIOD),
        )
        .await
        {
            let log = &mut logs[record.kind()];
            let Ok(row) = format_record(&record) else {
                warn!("SD: {} row exceeded buffer", log.name);
                continue;
            };
            if let Err(e) = log.append(row.as_bytes()).await {
                warn!("SD: {} write failed: {}", log.name, Debug2Format(&e));
                return;
            }
        }
        if Instant::now() < last_flush + FLUSH_PERIOD {
            continue;
        }

        let dropped: [u32; SdLogRecord::KIND_COUNT] =
            core::array::from_fn(|kind| SD_LOG_DROPPED[kind].swap(0, Ordering::Relaxed));
        if dropped.iter().any(|&count| count > 0) {
            let mut row = Row::new();
            write!(row, "{}", Instant::now().as_micros()).unwrap();
            for count in dropped {
                write!(row, ",{}", count).unwrap();
            }
            row.push('\n').unwrap();
            let log = &mut logs[DROP_FILE];
            if let Err(e) = log.append(row.as_bytes()).await {
                warn!("SD: {} write failed: {}", log.name, Debug2Format(&e));
                return;
            }
        }

        let mut rows = [0u32; FILE_COUNT];
        for (log, rows) in logs.iter_mut().zip(&mut rows) {
            match log.flush().await {
                Ok(count) => *rows = count,
                Err(e) => {
                    warn!("SD: {} flush failed: {}", log.name, Debug2Format(&e));
                    return;
                }
            }
        }
        info!(
            "SD: flushed state={}, IMU={}, mag={}, GNSS={}, baro={}, DHT={}, dropped={}",
            rows[0], rows[1], rows[2], rows[3], rows[4], rows[5], dropped,
        );
        last_flush = Instant::now();
    }
}

fn format_record(record: &SdLogRecord) -> Result<Row, core::fmt::Error> {
    let mut row = Row::new();
    match record {
        SdLogRecord::State(s) => {
            let [qw, qx, qy, qz] = s.orientation_body_to_ned_wxyz;
            write!(
                row,
                "{},{},{},{},{},{:.3},{:.4},{:.3},{:.3},{:.3},{:.4},{:.3},{:.3},{:.3},{:.6},{:.6},{:.6},{:.6}\n",
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
        SdLogRecord::Imu { raw, cal } => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            write!(
                row,
                ",{},{},{},{},{},{},{:.6},{:.6},{:.6},{:.4},{:.4},{:.4}\n",
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
        SdLogRecord::Mag { raw, cal } => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            write!(
                row,
                ",{},{},{},{:.2},{:.2},{:.2}\n",
                raw.x, raw.y, raw.z, cal.x, cal.y, cal.z,
            )?;
        }
        SdLogRecord::Gnss { raw, cal } => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            let p = cal.pvt;
            write!(
                row,
                ",{},{},{},{},{:.8},{:.8},{:.3},{:.4},{},{},{:.4},{}\n",
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
        SdLogRecord::Baro { raw, cal } => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            write!(row, ",{:.3},{:.3}\n", cal.pressure_mbar, cal.temperature_c)?;
        }
        SdLogRecord::Dht { raw, cal } => {
            write_times(&mut row, raw.read_ts, raw.ts, cal.ts, raw.src.index())?;
            write!(row, ",{:.2},{:.2}\n", cal.temperature_c, cal.humidity_rh)?;
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
