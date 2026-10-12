//! Full-rate CSV logging, with a new `LOGnnnn` directory per session.

mod csv;
mod writer;

use core::fmt::{Debug, Write as _};

use block_device_adapters::{BufStream, StreamSlice};
use defmt::{Debug2Format, debug, info, warn};
use embassy_futures::select::{Either, Either6, select, select6};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::sdmmc::sd::{Addressable, Card, CmdBlock, StorageDevice};
use embassy_stm32::time::mhz;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_fatfs::{
    DefaultTimeProvider, Error as FatError, FileSystem, FsOptions, LossyOemCpConverter,
};
use embedded_io_async::{Seek, Write};
use embedded_partitions::mbr::Mbr;
use heapless::{String, Vec};
use static_cell::ConstStaticCell;

use crate::resources::sd::Sd;
use crate::signals;
use csv::{DROP_FILE, FILE_COUNT, FILES, RECORD_KINDS, Record};
use writer::{CsvLog, LogBuffer};

const FLUSH_PERIOD: Duration = Duration::from_secs(1);

// Static buffers prevent stack overflow when the estimator interrupts SD logging.
static LOG_BUFFERS: ConstStaticCell<[LogBuffer; FILE_COUNT]> =
    ConstStaticCell::new([const { LogBuffer::new() }; FILE_COUNT]);

type SdFileSystem<'a> = FileSystem<
    StreamSlice<BufStream<StorageDevice<'a, 'static, Card>, 512>>,
    DefaultTimeProvider,
    LossyOemCpConverter,
>;

#[embassy_executor::task]
pub async fn task(mut sdmmc: Sd, detect: Input<'static>, _power: Output<'static>) {
    let buffers = LOG_BUFFERS.take();
    signals::SD_LOGGING.sender().send(false);
    debug!(
        "SD: card detect pin is {}",
        if detect.is_high() { "high" } else { "low" }
    );
    // Allow the switched SD supply to settle after setup.
    Timer::after(Duration::from_millis(250)).await;
    loop {
        let _ = run_session(&mut sdmmc, buffers).await;
        signals::SD_LOGGING.sender().send(false);
        Timer::after(Duration::from_secs(5)).await;
    }
}

async fn run_session(sdmmc: &mut Sd, buffers: &mut [LogBuffer; FILE_COUNT]) -> Result<(), ()> {
    let fs = mount(sdmmc).await?;
    let mut logs = create_logs(&fs, buffers).await?;

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
                Either::First(Either6::First(message)) => (0, received(message, Record::State)),
                Either::First(Either6::Second(message)) => (1, received(message, Record::Imu)),
                Either::First(Either6::Third(message)) => (2, received(message, Record::Mag)),
                Either::First(Either6::Fourth(message)) => (3, received(message, Record::Gnss)),
                Either::First(Either6::Fifth(message)) => (4, received(message, Record::Baro)),
                Either::First(Either6::Sixth(message)) => (5, received(message, Record::Dht)),
                Either::Second(message) => (6, received(message, Record::Mark)),
            };
            match received {
                Ok(record) => {
                    let log = &mut logs[kind];
                    let Ok(row) = csv::format_record(&record) else {
                        warn!("SD: {} row exceeded buffer", log.name);
                        continue;
                    };
                    let started = Instant::now();
                    log.append(row.as_bytes()).await.map_err(|e| {
                        warn!("SD: {} write failed: {}", log.name, Debug2Format(&e))
                    })?;
                    longest_write = longest_write.max(started.elapsed());
                }
                Err(lost) => dropped[kind] += lost,
            }
        }
        if Instant::now() < last_flush + FLUSH_PERIOD {
            continue;
        }

        if dropped.iter().any(|&count| count > 0) {
            let row = csv::format_drops(Instant::now(), &dropped);
            let log = &mut logs[DROP_FILE];
            log.append(row.as_bytes())
                .await
                .map_err(|e| warn!("SD: {} write failed: {}", log.name, Debug2Format(&e)))?;
        }

        let mut rows = [0u32; FILE_COUNT];
        let started = Instant::now();
        for (log, rows) in logs.iter_mut().zip(&mut rows) {
            *rows = log
                .flush()
                .await
                .map_err(|e| warn!("SD: {} flush failed: {}", log.name, Debug2Format(&e)))?;
        }
        longest_write = longest_write.max(started.elapsed());
        signals::SD_LOGGING.sender().send(true);
        if dropped.iter().any(|&count| count > 0) {
            warn!(
                "SD: dropped state/IMU/mag/GNSS/baro/DHT/mark readings: {}",
                dropped
            );
        }
        // Card writes must finish before the sample channels fill.
        debug!(
            "SD: flushed state={}, IMU={}, mag={}, GNSS={}, baro={}, DHT={}, dropped={}, longest card write={} ms",
            rows[0],
            rows[1],
            rows[2],
            rows[3],
            rows[4],
            rows[5],
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
    let fs = FileSystem::new(slice, FsOptions::new())
        .await
        .map_err(|e| warn!("SD: FAT mount failed: {}", Debug2Format(&e)))?;
    Ok(fs)
}

async fn create_logs<'buf>(
    fs: &SdFileSystem<'_>,
    buffers: &'buf mut [LogBuffer; FILE_COUNT],
) -> Result<Vec<CsvLog<'buf, impl Write<Error: Debug> + Seek>, FILE_COUNT>, ()> {
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
    for (spec, buffer) in FILES.iter().zip(buffers) {
        let file = dir
            .create_file(spec.name)
            .await
            .map_err(|e| warn!("SD: {} create failed: {}", spec.name, Debug2Format(&e)))?;
        let _ = logs.push(CsvLog::new(spec.name, spec.header, file, buffer));
    }
    info!("SD: logging to {}", dir_name.as_str());
    Ok(logs)
}

fn received<T: Clone>(message: WaitResult<T>, record: fn(T) -> Record) -> Result<Record, u64> {
    match message {
        WaitResult::Message(value) => Ok(record(value)),
        WaitResult::Lagged(lost) => Err(lost),
    }
}
