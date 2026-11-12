//! Full-rate binary logging, with a new `LOGnnnn` directory per session.

mod record;
mod writer;

use core::fmt::Write as _;

use block_device_adapters::{BufStream, StreamSlice};
use defmt::{Debug2Format, debug, info, warn};
use embassy_futures::select::{Either, Either6, select, select6};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::sdmmc::sd::{Addressable, Card, CmdBlock, StorageDevice};
use embassy_stm32::time::mhz;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_fatfs::{
    DefaultTimeProvider, Dir, Error as FatError, File, FileSystem, FsOptions, LossyOemCpConverter,
};
use embedded_io_async::{ErrorType, Write as _};
use embedded_partitions::mbr::Mbr;
use heapless::String;
use static_cell::ConstStaticCell;

use crate::resources::sd::Sd;
use crate::signals;
use crate::types::SdSession;
use record::{Kind, MAX_FRAME, RECORD_KINDS, Record};
use writer::{LogBuffer, SegmentLog};

const FLUSH_PERIOD: Duration = Duration::from_secs(1);

// A static buffer prevents stack overflow when the estimator interrupts SD logging.
static LOG_BUFFER: ConstStaticCell<LogBuffer> = ConstStaticCell::new(LogBuffer::new());
static SCHEMA: ConstStaticCell<[u8; 6144]> = ConstStaticCell::new([0; 6144]);

type SdIo<'sd> = StreamSlice<BufStream<StorageDevice<'sd, 'static, Card>, 512>>;
type SdFileSystem<'sd> = FileSystem<SdIo<'sd>, DefaultTimeProvider, LossyOemCpConverter>;
type SdDir<'fs, 'sd> = Dir<'fs, SdIo<'sd>, DefaultTimeProvider, LossyOemCpConverter>;
type SdFile<'fs, 'sd> = File<'fs, SdIo<'sd>, DefaultTimeProvider, LossyOemCpConverter>;
type SdError<'sd> = FatError<<SdIo<'sd> as ErrorType>::Error>;

#[embassy_executor::task]
pub async fn task(mut sdmmc: Sd, detect: Input<'static>, _power: Output<'static>) {
    let buffer = LOG_BUFFER.take();
    let schema = record::schema_json(SCHEMA.take());
    // Describe the session next to its data.
    let files: [(&str, &[u8]); 1] = [("SCHEMA.TXT", schema)];
    signals::SD_LOGGING.sender().send(false);
    debug!(
        "SD: card detect pin is {}",
        if detect.is_high() { "high" } else { "low" }
    );
    // Allow the switched SD supply to settle after setup.
    Timer::after(Duration::from_millis(250)).await;
    // Outlives card errors, so logging resumes in the same directory.
    let mut log_dir = None;
    loop {
        let _ = run_session(&mut sdmmc, buffer, &files, &mut log_dir).await;
        signals::SD_LOGGING.sender().send(false);
        Timer::after(Duration::from_secs(5)).await;
    }
}

async fn run_session(
    sdmmc: &mut Sd,
    buffer: &mut LogBuffer,
    files: &[(&str, &[u8])],
    log_dir: &mut Option<String<8>>,
) -> Result<(), ()> {
    let fs = mount(sdmmc).await?;
    let name = match log_dir {
        Some(name) => name.clone(),
        None => next_log_dir(&fs).await?,
    };
    *log_dir = Some(name.clone());
    let dir = fs
        .root_dir()
        .create_dir(&name)
        .await
        .map_err(|e| warn!("SD: log directory create failed: {}", Debug2Format(&e)))?;
    for (file_name, contents) in files {
        write_file(&dir, file_name, contents).await?;
    }
    let mut log = SegmentLog::new(dir, buffer)
        .await
        .map_err(|e| warn!("SD: segment create failed: {}", Debug2Format(&e)))?;
    info!("SD: logging to {}", name.as_str());

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
    let mut mark = signals::MARK_CHANNEL
        .subscriber()
        .expect("SD: subscriber slot");
    let mut dropped = [0u64; RECORD_KINDS];
    let mut session_dropped = 0;
    let mut longest_write = Duration::from_ticks(0);
    let mut last_flush = Instant::now();
    loop {
        let sensors = select6(
            state.next_message(),
            imu.next_message(),
            mag.next_message(),
            gnss.next_message(),
            baro.next_message(),
            dht.next_message(),
        );
        let next = select(sensors, mark.next_message());
        if let Either::First(next) = select(next, Timer::at(last_flush + FLUSH_PERIOD)).await {
            let (kind, received) = match next {
                Either::First(Either6::First(message)) => (Kind::State, received(message)),
                Either::First(Either6::Second(message)) => (Kind::Imu, received(message)),
                Either::First(Either6::Third(message)) => (Kind::Mag, received(message)),
                Either::First(Either6::Fourth(message)) => (Kind::Gnss, received(message)),
                Either::First(Either6::Fifth(message)) => (Kind::Baro, received(message)),
                Either::First(Either6::Sixth(message)) => (Kind::Dht, received(message)),
                Either::Second(message) => (Kind::Marks, received(message)),
            };
            match received {
                Ok(record) => {
                    let started = Instant::now();
                    let mut frame = [0; MAX_FRAME];
                    log.append(record.encode(&mut frame))
                        .await
                        .map_err(|e| warn!("SD: write failed: {}", Debug2Format(&e)))?;
                    longest_write = longest_write.max(started.elapsed());
                }
                Err(lost) => dropped[kind as usize] += lost,
            }
        }
        if Instant::now() < last_flush + FLUSH_PERIOD {
            continue;
        }

        let lost = dropped.iter().sum::<u64>();
        if lost > 0 {
            let record = Record::drops(Instant::now(), &dropped);
            let mut frame = [0; MAX_FRAME];
            log.append(record.encode(&mut frame))
                .await
                .map_err(|e| warn!("SD: write failed: {}", Debug2Format(&e)))?;
        }

        let started = Instant::now();
        log.flush()
            .await
            .map_err(|e| warn!("SD: flush failed: {}", Debug2Format(&e)))?;
        longest_write = longest_write.max(started.elapsed());
        session_dropped += lost;
        signals::SD_SESSION.sender().send(SdSession {
            log: name.clone(),
            dropped: session_dropped,
        });
        signals::SD_LOGGING.sender().send(true);
        if lost > 0 {
            warn!(
                "SD: dropped state/IMU/mag/GNSS/baro/DHT/mark readings: {}",
                dropped
            );
        }
        // Card writes must finish before the sample channels fill.
        debug!(
            "SD: flushed, dropped={}, longest card write={} ms",
            dropped,
            longest_write.as_millis(),
        );
        dropped = [0; RECORD_KINDS];
        longest_write = Duration::from_ticks(0);
        last_flush = Instant::now();
    }
}

async fn mount(sdmmc: &mut Sd) -> Result<SdFileSystem<'_>, ()> {
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
    FileSystem::new(slice, FsOptions::new())
        .await
        .map_err(|e| warn!("SD: FAT mount failed: {}", Debug2Format(&e)))
}

async fn next_log_dir(fs: &SdFileSystem<'_>) -> Result<String<8>, ()> {
    let root = fs.root_dir();
    let mut dir_name = String::<8>::new();
    for number in 1..=9999 {
        dir_name.clear();
        write!(dir_name, "LOG{:04}", number).unwrap();
        match root.open_dir(&dir_name).await {
            Ok(_) => continue,
            Err(FatError::NotFound) => return Ok(dir_name),
            Err(e) => {
                warn!("SD: log directory lookup failed: {}", Debug2Format(&e));
                return Err(());
            }
        }
    }
    warn!("SD: all log directory names are occupied");
    Err(())
}

async fn write_file(dir: &SdDir<'_, '_>, name: &str, contents: &[u8]) -> Result<(), ()> {
    let mut file = dir
        .create_file(name)
        .await
        .map_err(|e| warn!("SD: {} create failed: {}", name, Debug2Format(&e)))?;
    file.write_all(contents)
        .await
        .map_err(|e| warn!("SD: {} write failed: {}", name, Debug2Format(&e)))?;
    file.flush()
        .await
        .map_err(|e| warn!("SD: {} write failed: {}", name, Debug2Format(&e)))
}

fn received<T: Clone + Into<Record>>(message: WaitResult<T>) -> Result<Record, u64> {
    match message {
        WaitResult::Message(value) => Ok(value.into()),
        WaitResult::Lagged(lost) => Err(lost),
    }
}
