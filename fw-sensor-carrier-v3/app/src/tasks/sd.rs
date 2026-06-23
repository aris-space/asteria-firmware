//! Binary data logger to the SD card.
//!
//! Producer tasks subscribe to the raw IMU and GNSS streams and push each
//! sample into [`LOG_CHANNEL`]; the state-estimation task pushes the EKF state
//! (every predict) and GNSS-correction diagnostics directly via
//! [`log_ekf_state`] / [`log_gnss_correction`]. The single writer [`task`]
//! mounts the first FAT partition, opens a fresh `LOGNNNNN.BIN` file, and drains
//! the channel into a staging buffer that is flushed to the card in chunks.
//!
//! The channel decouples the high-rate sensor streams from SD write latency:
//! producers `try_send` and never block a sensor subscriber, so a slow flush
//! drops records (counted in [`DROPPED`]) rather than stalling the pipeline.
//!
//! ## File format
//!
//! 8-byte header `b"SCV3LOG\x01"` followed by a stream of records. Each record
//! is `0xA5 0x5A <tag:u8> <payload>`, little-endian, fixed layout per tag:
//!
//! * `0x01` IMU (raw): `sensor_id:u8, ts_us:u64, accel_xyz:f32×3 (g),
//!   gyro_xyz:f32×3 (dps)`
//! * `0x02` GNSS (raw): `sensor_id:u8, ts_us:u64, lat:f64, lon:f64, fix:u8,
//!   sats:u8, h_ellipsoid:f32, h_msl:f32, heading:f32, heading_acc:f32,
//!   heading_veh:f32, vn:f32, ve:f32, vd:f32, pdop:u16, v_acc:u32, h_acc:u32,
//!   mag_decl:f32, mag_decl_acc:f32`
//! * `0x03` EKF state: `ts_us:u64, pos_ned:f32×3, vel_ned:f32×3,
//!   quat_wxyz:f32×4, cov_diag:f32×9, predict_us:f32` (predict step duration)
//! * `0x04` GNSS correction: `ts_us:u64, residual:f32×3 (m), innovation_cov:f32×3
//!   (m²), adapted_var:f32×3 (m², NaN when not adaptive), correct_us:f32`
//!   (correct step duration) — diagnostics from the EKF GNSS correction step
//!   (NaN where a scalar was rejected / unavailable).
//!
//! See `scripts/decode_log.py` for a reference decoder.

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use block_device_adapters::BufStream;
use defmt::{Debug2Format, error, info, warn};
use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::sdmmc::sd::{Addressable, CmdBlock, StorageDevice};
use embassy_stm32::time::mhz;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_fatfs::{Error as FatError, FileSystem, FsOptions};
use embedded_io_async::Write;
use embedded_partitions::mbr::Mbr;

use crate::measurements::{GnssSample, ImuSample, StateEstimate};
use crate::resources::sd::Sd;
use crate::sensors::{GnssId, ImuId};
use crate::signals;

/// File header magic + format version. Written into the first [`SECTOR`] so all
/// subsequent record writes stay sector-aligned (avoids read-modify-write).
const FILE_MAGIC: &[u8] = b"SCV3LOG\x01";

const SYNC0: u8 = 0xA5;
const SYNC1: u8 = 0x5A;
const TAG_IMU: u8 = 0x01;
const TAG_GNSS: u8 = 0x02;
const TAG_EKF: u8 = 0x03;
const TAG_GNSS_CORR: u8 = 0x04;

/// SD block size. Keeping every write a multiple of this, and the stream
/// position sector-aligned, keeps the card off the read-modify-write path.
const SECTOR: usize = 512;
/// Largest possible encoded record. Largest payload is now EKF (88 B) or GNSS
/// raw (77 B); 96 leaves margin and keeps the staging-headroom assert simple.
const MAX_RECORD: usize = 96;
/// In-RAM staging buffer. Sized so a full flush chunk plus carry-over fits.
const STAGING_LEN: usize = 16384;
/// Write whole sectors out once staged data reaches this many bytes.
const FLUSH_THRESHOLD: usize = 8192;
/// Depth of the producer-writer queue. Each slot is one `LogRecord`; deep
/// enough to ride out occasional multi-ms card write stalls without dropping.
const LOG_QUEUE_DEPTH: usize = 1024;

// Staging must hold a full flush chunk plus one more record before the next
// flush, so a single encode can never overrun the buffer.
const _: () = assert!(FLUSH_THRESHOLD + MAX_RECORD <= STAGING_LEN);

/// How often to flush buffered data through to the card (bounds data loss on
/// power-off to this interval).
const FLUSH_INTERVAL: Duration = Duration::from_secs(1);
/// Wake the writer to flush even if no records arrive.
const IDLE_FLUSH: Duration = Duration::from_millis(500);
/// How often to log throughput / drop stats.
const STATS_INTERVAL: Duration = Duration::from_secs(5);

/// Diagnostics from one EKF GNSS correction step. Each per-axis value is `NaN`
/// when the scalar was rejected or the figure is unavailable (e.g. `adapted_var`
/// for a non-adaptive corrector).
#[derive(Clone, Copy)]
pub struct GnssCorr {
    pub ts_us: u64,
    /// Residual
    pub residual: [f32; 3],
    /// Innovation covariance
    pub innovation_cov: [f32; 3],
    /// Adaptive noise estimate
    pub adapted_var: [f32; 3],
    /// Duration of the EKF correct step
    pub correct_us: f32,
}

#[derive(Clone, Copy)]
pub enum LogRecord {
    Imu(ImuSample),
    Gnss(GnssSample),
    Ekf {
        state: StateEstimate,
        /// Duration of the EKF predict step
        predict_us: f32,
    },
    GnssCorr(GnssCorr),
}

/// Enqueue an EKF state snapshot plus the predict-step duration (called from the
/// state estimation task on every predict step).
pub fn log_ekf_state(state: StateEstimate, predict_us: f32) {
    push(LogRecord::Ekf { state, predict_us });
}

/// Enqueue an EKF GNSS-correction diagnostics record (called from the state
/// estimation task after each correction).
pub fn log_gnss_correction(corr: GnssCorr) {
    push(LogRecord::GnssCorr(corr));
}

static LOG_CHANNEL: Channel<CriticalSectionRawMutex, LogRecord, LOG_QUEUE_DEPTH> = Channel::new();
/// Records dropped because the queue was full (writer couldn't keep up).
static DROPPED: AtomicU32 = AtomicU32::new(0);
/// Set once the writer has opened the log file. Until then producers drain
/// their sensor streams but discard, so the queue isn't flooded during the
/// (slow) SD init + FAT mount.
static READY: AtomicBool = AtomicBool::new(false);

fn push(rec: LogRecord) {
    if !READY.load(Ordering::Relaxed) {
        return;
    }
    if LOG_CHANNEL.try_send(rec).is_err() {
        DROPPED.fetch_add(1, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
pub async fn imu_producer(id: ImuId) -> ! {
    let mut sub = signals::IMU_CHANNELS[id.index()].subscriber().unwrap();
    loop {
        let sample = sub.next_message_pure().await;
        push(LogRecord::Imu(sample));
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn gnss_producer(id: GnssId) -> ! {
    let mut sub = signals::GNSS_CHANNELS[id.index()].subscriber().unwrap();
    loop {
        let sample = sub.next_message_pure().await;
        push(LogRecord::Gnss(sample));
    }
}

/// Little-endian record encoder over a caller-provided slice.
struct Encoder<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Encoder<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    fn u8(&mut self, v: u8) {
        self.buf[self.pos] = v;
        self.pos += 1;
    }
    fn u16(&mut self, v: u16) {
        self.buf[self.pos..self.pos + 2].copy_from_slice(&v.to_le_bytes());
        self.pos += 2;
    }
    fn u32(&mut self, v: u32) {
        self.buf[self.pos..self.pos + 4].copy_from_slice(&v.to_le_bytes());
        self.pos += 4;
    }
    fn u64(&mut self, v: u64) {
        self.buf[self.pos..self.pos + 8].copy_from_slice(&v.to_le_bytes());
        self.pos += 8;
    }
    fn f32(&mut self, v: f32) {
        self.buf[self.pos..self.pos + 4].copy_from_slice(&v.to_le_bytes());
        self.pos += 4;
    }
    fn f64(&mut self, v: f64) {
        self.buf[self.pos..self.pos + 8].copy_from_slice(&v.to_le_bytes());
        self.pos += 8;
    }
}

/// Encode one record (frame + payload) into `out`. Returns bytes written.
/// `out` must have at least [`MAX_RECORD`] bytes of room.
fn encode_record(rec: &LogRecord, out: &mut [u8]) -> usize {
    let mut e = Encoder::new(out);
    e.u8(SYNC0);
    e.u8(SYNC1);
    match rec {
        LogRecord::Imu(s) => {
            e.u8(TAG_IMU);
            e.u8(s.sensor_id.index() as u8);
            e.u64(s.data.ts.as_micros());
            let a = &s.data.value.accel;
            let g = &s.data.value.gyro;
            e.f32(a.x);
            e.f32(a.y);
            e.f32(a.z);
            e.f32(g.x);
            e.f32(g.y);
            e.f32(g.z);
        }
        LogRecord::Gnss(s) => {
            e.u8(TAG_GNSS);
            e.u8(s.sensor_id.index() as u8);
            e.u64(s.data.ts.as_micros());
            let p = &s.data.value;
            e.f64(p.lat_deg);
            e.f64(p.lon_deg);
            e.u8(p.fix_type as u8);
            e.u8(p.num_satellites);
            e.f32(p.height_ellipsoid_m);
            e.f32(p.height_msl);
            e.f32(p.heading_deg);
            e.f32(p.heading_accuracy_estimate);
            e.f32(p.heading_of_vehicle_deg);
            e.f32(p.vel_north);
            e.f32(p.vel_east);
            e.f32(p.vel_down);
            e.u16(p.pdop);
            e.u32(p.vert_accuracy);
            e.u32(p.horiz_accuracy);
            e.f32(p.magnetic_declination_deg);
            e.f32(p.magnetic_declination_accuracy_deg);
        }
        LogRecord::Ekf {
            state: s,
            predict_us,
        } => {
            // State is f64 internally but logged as f32 to halve the record
            // size — mm position / ~1e-7 quaternion precision is ample here.
            e.u8(TAG_EKF);
            e.u64(s.ts.as_micros());
            for v in s.pos_ned_m {
                e.f32(v as f32);
            }
            for v in s.vel_ned_mps {
                e.f32(v as f32);
            }
            for v in s.attitude_quat_wxyz {
                e.f32(v as f32);
            }
            for v in s.cov_diag {
                e.f32(v as f32);
            }
            e.f32(*predict_us);
        }
        LogRecord::GnssCorr(c) => {
            e.u8(TAG_GNSS_CORR);
            e.u64(c.ts_us);
            for v in c.residual {
                e.f32(v);
            }
            for v in c.innovation_cov {
                e.f32(v);
            }
            for v in c.adapted_var {
                e.f32(v);
            }
            e.f32(c.correct_us);
        }
    }
    e.pos
}

/// Park forever, keeping the task frame (and thus the powered SD peripheral)
/// alive. Used on unrecoverable setup failures.
async fn park() -> ! {
    loop {
        core::future::pending::<()>().await;
    }
}

#[embassy_executor::task]
pub async fn task(mut sdmmc: Sd, detect: Input<'static>, _power: Output<'static>) -> ! {
    info!(
        "sd-log: detect = {}",
        if detect.is_high() { "high" } else { "low" }
    );

    // SD_VDD (PD6) is on from setup(); let the rail settle.
    Timer::after(Duration::from_millis(250)).await;

    let mut cmd_block = CmdBlock::new();
    let card = match with_timeout(
        Duration::from_secs(2),
        StorageDevice::new_sd_card(&mut sdmmc, &mut cmd_block, mhz(25)),
    )
    .await
    {
        Ok(Ok(card)) => card,
        Ok(Err(e)) => {
            error!("sd-log: init failed: {}", Debug2Format(&e));
            park().await;
        }
        Err(_) => {
            error!("sd-log: init timed out (card seated / powered?)");
            park().await;
        }
    };
    info!(
        "sd-log: init OK ({} MiB)",
        card.card().size() / (1024 * 1024)
    );

    let mbr = match Mbr::new(BufStream::<_, 512>::new(card)).await {
        Ok(m) => m,
        Err(e) => {
            error!("sd-log: read MBR failed: {}", Debug2Format(&e));
            park().await;
        }
    };
    let Some(idx) = mbr.iter_used().find(|(_, p)| p.is_fat()).map(|(i, _)| i) else {
        error!("sd-log: no FAT partition found (is the card FAT-formatted?)");
        park().await;
    };
    let slice = match mbr.into_partition(idx).await {
        Ok(s) => s,
        Err(e) => {
            error!("sd-log: open partition failed: {}", Debug2Format(&e));
            park().await;
        }
    };
    let fs = match FileSystem::new(slice, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(e) => {
            error!("sd-log: mount FAT failed: {}", Debug2Format(&e));
            park().await;
        }
    };
    let root = fs.root_dir();

    // Pick the next free LOGNNNNN.BIN so each boot writes a fresh file.
    let mut name = heapless::String::<16>::new();
    let mut n: u32 = 0;
    loop {
        name.clear();
        let _ = write!(name, "LOG{:05}.BIN", n);
        match root.open_file(name.as_str()).await {
            Ok(_) => {
                n += 1;
                if n > 99_999 {
                    error!("sd-log: log filenames exhausted");
                    park().await;
                }
            }
            Err(FatError::NotFound) => break,
            Err(e) => {
                error!("sd-log: directory scan failed: {}", Debug2Format(&e));
                park().await;
            }
        }
    }

    let mut file = match root.create_file(name.as_str()).await {
        Ok(f) => f,
        Err(e) => {
            error!("sd-log: create file failed: {}", Debug2Format(&e));
            park().await;
        }
    };
    if let Err(e) = file.truncate().await {
        error!("sd-log: truncate failed: {}", Debug2Format(&e));
        park().await;
    }
    // Sector-sized header so record data begins at a 512-byte boundary.
    let mut header = [0u8; SECTOR];
    header[..FILE_MAGIC.len()].copy_from_slice(FILE_MAGIC);
    if let Err(e) = file.write_all(&header).await {
        error!("sd-log: header write failed: {}", Debug2Format(&e));
        park().await;
    }
    info!("sd-log: writing {}", name.as_str());

    // Open the floodgates: producers may now enqueue.
    READY.store(true, Ordering::Relaxed);

    let mut staging = [0u8; STAGING_LEN];
    let mut len = 0usize;
    let mut bytes_total: u64 = 0;
    let mut last_flush = Instant::now();
    let mut last_stats = Instant::now();

    loop {
        if let Either::First(rec) = select(LOG_CHANNEL.receive(), Timer::after(IDLE_FLUSH)).await {
            len += encode_record(&rec, &mut staging[len..]);
            if len >= FLUSH_THRESHOLD {
                write_sectors(&mut file, &mut staging, &mut len, &mut bytes_total).await;
            }
        }

        let now = Instant::now();
        if now >= last_flush + FLUSH_INTERVAL {
            write_sectors(&mut file, &mut staging, &mut len, &mut bytes_total).await;
            if let Err(e) = file.flush().await {
                warn!("sd-log: flush failed: {}", Debug2Format(&e));
            }
            last_flush = now;
        }
        if now >= last_stats + STATS_INTERVAL {
            info!(
                "sd-log: {} KiB written, {} records dropped",
                bytes_total / 1024,
                DROPPED.load(Ordering::Relaxed),
            );
            last_stats = now;
        }
    }
}

/// Write out as many whole [`SECTOR`]s as `staging` holds, then shift the
/// sub-sector remainder to the front. Keeps every card write sector-aligned.
async fn write_sectors<F: Write>(
    file: &mut F,
    staging: &mut [u8],
    len: &mut usize,
    bytes_total: &mut u64,
) {
    let whole = (*len / SECTOR) * SECTOR;
    if whole == 0 {
        return;
    }
    if let Err(e) = file.write_all(&staging[..whole]).await {
        warn!("sd-log: write failed: {}", Debug2Format(&e));
    }
    *bytes_total += whole as u64;
    staging.copy_within(whole..*len, 0);
    *len -= whole;
}
