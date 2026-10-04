//! Persist the published state estimate on the first FAT partition of the SD card.

use core::fmt::Write as _;

use block_device_adapters::BufStream;
use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, select};
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::sdmmc::sd::{Addressable, CmdBlock, StorageDevice};
use embassy_stm32::time::mhz;
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_fatfs::{Error as FatError, FileSystem, FsOptions};
use embedded_io_async::{Read, Seek, SeekFrom, Write};
use embedded_partitions::mbr::Mbr;
use heapless::String;

use crate::resources::sd::Sd;
use crate::signals::STATE_ESTIMATE_WATCH;
use crate::types::StateEstimate;

const HEADER: &[u8] = b"uptime_ms,msl_ready,height_msl_m,velocity_mps,height_std_m,velocity_std_mps,qw,qx,qy,qz,selected_imu,redundancy_ready\n";
const LOG_PERIOD: Duration = Duration::from_millis(250);
const FLUSH_PERIOD: Duration = Duration::from_secs(1);

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
    let mut filename = String::<16>::new();
    let mut found = false;
    for number in 1..=9999 {
        filename.clear();
        write!(&mut filename, "LOG{:04}.CSV", number).unwrap();
        match root.open_file(&filename).await {
            Ok(_) => continue,
            Err(FatError::NotFound) => {
                found = true;
                break;
            }
            Err(e) => {
                warn!("SD: log name lookup failed: {}", Debug2Format(&e));
                return;
            }
        }
    }
    if !found {
        warn!("SD: all log filenames are occupied");
        return;
    }
    let mut file = match root.create_file(&filename).await {
        Ok(file) => file,
        Err(e) => {
            warn!("SD: log create failed: {}", Debug2Format(&e));
            return;
        }
    };
    if let Err(e) = file.write_all(HEADER).await {
        warn!("SD: header write failed: {}", Debug2Format(&e));
        return;
    }
    if let Err(e) = file.flush().await {
        warn!("SD: header flush failed: {}", Debug2Format(&e));
        return;
    }
    if let Err(e) = file.seek(SeekFrom::Start(0)).await {
        warn!("SD: header seek failed: {}", Debug2Format(&e));
        return;
    }
    let mut readback = [0u8; HEADER.len()];
    if let Err(e) = file.read_exact(&mut readback).await {
        warn!("SD: header readback failed: {}", Debug2Format(&e));
        return;
    }
    if readback != *HEADER {
        warn!("SD: header readback mismatch");
        return;
    }
    if let Err(e) = file.seek(SeekFrom::End(0)).await {
        warn!("SD: log seek failed: {}", Debug2Format(&e));
        return;
    }
    info!("SD: logging to {} (header verified)", filename.as_str());

    let mut receiver = STATE_ESTIMATE_WATCH.receiver().expect("SD watch receiver");
    let mut last_write = None;
    let mut last_flush = Instant::now();
    let mut rows_since_flush = 0u32;
    loop {
        if let Either::First(estimate) =
            select(receiver.changed(), Timer::after(FLUSH_PERIOD)).await
        {
            let now = Instant::now();
            if last_write
                .is_none_or(|last: Instant| now.saturating_duration_since(last) >= LOG_PERIOD)
            {
                let Some(row) = format_row(estimate) else {
                    warn!("SD: state row exceeded log buffer");
                    continue;
                };
                if let Err(e) = file.write_all(row.as_bytes()).await {
                    warn!("SD: row write failed: {}", Debug2Format(&e));
                    return;
                }
                last_write = Some(now);
                rows_since_flush += 1;
            }
        }
        if rows_since_flush > 0
            && Instant::now().saturating_duration_since(last_flush) >= FLUSH_PERIOD
        {
            if let Err(e) = file.flush().await {
                warn!("SD: log flush failed: {}", Debug2Format(&e));
                return;
            }
            info!(
                "SD: flushed {} rows to {}",
                rows_since_flush,
                filename.as_str()
            );
            rows_since_flush = 0;
            last_flush = Instant::now();
        }
    }
}

fn format_row(estimate: StateEstimate) -> Option<String<512>> {
    let mut row = String::new();
    let [w, x, y, z] = estimate.orientation_body_to_ned_wxyz;
    write!(
        &mut row,
        "{},{},{:.3},{:.4},{:.3},{:.4},{:.6},{:.6},{:.6},{:.6},{},{}\n",
        estimate.ts.as_millis(),
        u8::from(estimate.msl_ready),
        estimate.height_msl_m,
        estimate.velocity_mps,
        estimate.height_std_m,
        estimate.velocity_std_mps,
        w,
        x,
        y,
        z,
        estimate.selected_imu.index(),
        u8::from(estimate.redundancy_ready),
    )
    .ok()?;
    Some(row)
}
