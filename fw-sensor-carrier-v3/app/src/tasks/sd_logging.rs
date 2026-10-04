//! Full-rate CSV logging through the async SDMMC and FAT drivers.

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
use embedded_io_async::{Read, Seek, SeekFrom, Write};
use embedded_partitions::mbr::Mbr;
use heapless::String;

use crate::resources::sd::Sd;
use crate::signals::{SD_LOG_CHANNEL, SD_LOG_DROPPED};
use crate::types::{SdLogRecord, SefLogSample};

const FILE_COUNT: usize = 6;
const PREFIXES: [char; FILE_COUNT] = ['S', 'I', 'M', 'G', 'B', 'L'];
const HEADERS: [&[u8]; FILE_COUNT] = [
    b"uptime_us,imu,selected,msl_ready,redundancy_ready,selected_gnss,height_msl_m,velocity_mps,bias0_m,bias1_m,height_std_m,velocity_std_mps,bias0_std_m,bias1_std_m,score,qw,qx,qy,qz\n",
    b"uptime_us,imu,ax_mps2,ay_mps2,az_mps2,gx_radps,gy_radps,gz_radps\n",
    b"uptime_us,magnetometer,x_nt,y_nt,z_nt\n",
    b"uptime_us,receiver,fix_type,height_msl_m,velocity_down_mps,pdop_centi,vertical_accuracy_mm,speed_accuracy_mps\n",
    b"uptime_us,barometer,pressure_mbar,temperature_c\n",
    b"uptime_us,dropped_state,dropped_imu,dropped_magnetometer,dropped_gnss,dropped_barometer\n",
];
const BUFFER_SIZE: usize = 4096;
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
    let mut names = [const { String::<16>::new() }; FILE_COUNT];
    let mut found = false;
    for number in 1..=9999 {
        let mut occupied = false;
        for (index, prefix) in PREFIXES.into_iter().enumerate() {
            names[index].clear();
            write!(&mut names[index], "{}{:04}.CSV", prefix, number).unwrap();
            match root.open_file(names[index].as_str()).await {
                Ok(_) => occupied = true,
                Err(FatError::NotFound) => {}
                Err(e) => {
                    warn!("SD: log name lookup failed: {}", Debug2Format(&e));
                    return;
                }
            }
        }
        if !occupied {
            found = true;
            break;
        }
    }
    if !found {
        warn!("SD: all log filenames are occupied");
        return;
    }

    macro_rules! create_log {
        ($index:expr) => {{
            match root.create_file(names[$index].as_str()).await {
                Ok(file) => file,
                Err(e) => {
                    warn!(
                        "SD: {} create failed: {}",
                        names[$index].as_str(),
                        Debug2Format(&e)
                    );
                    return;
                }
            }
        }};
    }
    let mut files = [
        create_log!(0),
        create_log!(1),
        create_log!(2),
        create_log!(3),
        create_log!(4),
        create_log!(5),
    ];
    let mut readback = [0u8; 256];
    for (index, file) in files.iter_mut().enumerate() {
        let header = HEADERS[index];
        if let Err(e) = file.write_all(header).await {
            warn!(
                "SD: {} header write failed: {}",
                names[index].as_str(),
                Debug2Format(&e)
            );
            return;
        }
        if let Err(e) = file.flush().await {
            warn!(
                "SD: {} header flush failed: {}",
                names[index].as_str(),
                Debug2Format(&e)
            );
            return;
        }
        if let Err(e) = file.seek(SeekFrom::Start(0)).await {
            warn!(
                "SD: {} header seek failed: {}",
                names[index].as_str(),
                Debug2Format(&e)
            );
            return;
        }
        if let Err(e) = file.read_exact(&mut readback[..header.len()]).await {
            warn!(
                "SD: {} header readback failed: {}",
                names[index].as_str(),
                Debug2Format(&e)
            );
            return;
        }
        if readback[..header.len()] != *header {
            warn!("SD: {} header readback mismatch", names[index].as_str());
            return;
        }
        if let Err(e) = file.seek(SeekFrom::End(0)).await {
            warn!(
                "SD: {} seek failed: {}",
                names[index].as_str(),
                Debug2Format(&e)
            );
            return;
        }
    }
    info!("SD: CSV session {} ready", names[0].as_str());

    let mut buffers = [[0u8; BUFFER_SIZE]; FILE_COUNT];
    let mut used = [0usize; FILE_COUNT];
    let mut rows = [0u32; FILE_COUNT];
    let mut last_flush = Instant::now();
    loop {
        if let Either::First(record) =
            select(SD_LOG_CHANNEL.receive(), Timer::after(FLUSH_PERIOD)).await
        {
            let Some((index, row)) = format_record(record) else {
                warn!("SD: CSV row exceeded buffer");
                continue;
            };
            let bytes = row.as_bytes();
            if used[index] + bytes.len() > BUFFER_SIZE {
                if let Err(e) = files[index].write_all(&buffers[index][..used[index]]).await {
                    warn!(
                        "SD: {} write failed: {}",
                        names[index].as_str(),
                        Debug2Format(&e)
                    );
                    return;
                }
                used[index] = 0;
            }
            buffers[index][used[index]..used[index] + bytes.len()].copy_from_slice(bytes);
            used[index] += bytes.len();
            rows[index] += 1;
        }
        if Instant::now().saturating_duration_since(last_flush) >= FLUSH_PERIOD {
            let dropped: [u32; 5] =
                core::array::from_fn(|i| SD_LOG_DROPPED[i].swap(0, Ordering::Relaxed));
            if dropped.iter().any(|&count| count > 0) {
                let mut line = String::<128>::new();
                write!(
                    &mut line,
                    "{},{},{},{},{},{}\n",
                    Instant::now().as_micros(),
                    dropped[0],
                    dropped[1],
                    dropped[2],
                    dropped[3],
                    dropped[4]
                )
                .unwrap();
                let index = 5;
                let bytes = line.as_bytes();
                if used[index] + bytes.len() > BUFFER_SIZE {
                    if let Err(e) = files[index].write_all(&buffers[index][..used[index]]).await {
                        warn!("SD: loss log write failed: {}", Debug2Format(&e));
                        return;
                    }
                    used[index] = 0;
                }
                buffers[index][used[index]..used[index] + bytes.len()].copy_from_slice(bytes);
                used[index] += bytes.len();
                rows[index] += 1;
            }
            for index in 0..FILE_COUNT {
                if used[index] > 0 {
                    if let Err(e) = files[index].write_all(&buffers[index][..used[index]]).await {
                        warn!(
                            "SD: {} write failed: {}",
                            names[index].as_str(),
                            Debug2Format(&e)
                        );
                        return;
                    }
                    used[index] = 0;
                }
                if rows[index] > 0 {
                    if let Err(e) = files[index].flush().await {
                        warn!(
                            "SD: {} flush failed: {}",
                            names[index].as_str(),
                            Debug2Format(&e)
                        );
                        return;
                    }
                }
            }
            info!(
                "SD: flushed state={}, IMU={}, mag={}, GNSS={}, baro={}, dropped=[{},{},{},{},{}]",
                rows[0],
                rows[1],
                rows[2],
                rows[3],
                rows[4],
                dropped[0],
                dropped[1],
                dropped[2],
                dropped[3],
                dropped[4],
            );
            rows = [0; FILE_COUNT];
            last_flush = Instant::now();
        }
    }
}

fn format_record(record: SdLogRecord) -> Option<(usize, String<512>)> {
    let mut row = String::new();
    let index = match record {
        SdLogRecord::State(sample) => {
            format_state(&mut row, sample).ok()?;
            0
        }
        SdLogRecord::Imu(sample) => {
            write!(
                &mut row,
                "{},{},{:.5},{:.5},{:.5},{:.6},{:.6},{:.6}\n",
                sample.ts.as_micros(),
                sample.src.index(),
                sample.accel.x,
                sample.accel.y,
                sample.accel.z,
                sample.gyro.x,
                sample.gyro.y,
                sample.gyro.z,
            )
            .ok()?;
            1
        }
        SdLogRecord::Magnetometer(sample) => {
            write!(
                &mut row,
                "{},{},{:.2},{:.2},{:.2}\n",
                sample.ts.as_micros(),
                sample.src.index(),
                sample.x,
                sample.y,
                sample.z,
            )
            .ok()?;
            2
        }
        SdLogRecord::Gnss(sample) => {
            write!(
                &mut row,
                "{},{},{},{:.3},{:.4},{},{},{:.4}\n",
                sample.ts.as_micros(),
                sample.src.index(),
                sample.pvt.fix_type as u8,
                sample.pvt.height_msl,
                sample.pvt.vel_down,
                sample.pvt.pdop,
                sample.pvt.vert_accuracy,
                sample.pvt.speed_accuracy_mps,
            )
            .ok()?;
            3
        }
        SdLogRecord::Barometer(sample) => {
            write!(
                &mut row,
                "{},{},{:.3},{:.3}\n",
                sample.ts.as_micros(),
                sample.src.index(),
                sample.pressure_mbar,
                sample.temperature_c,
            )
            .ok()?;
            4
        }
    };
    Some((index, row))
}

fn format_state(row: &mut String<512>, sample: SefLogSample) -> core::fmt::Result {
    let [w, x, y, z] = sample.orientation_body_to_ned_wxyz;
    write!(
        row,
        "{},{},{},{},{},{},{:.3},{:.4},{:.3},{:.3},{:.3},{:.4},{:.3},{:.3},{:.3},{:.6},{:.6},{:.6},{:.6}\n",
        sample.ts.as_micros(),
        sample.imu.index(),
        u8::from(sample.selected),
        u8::from(sample.msl_ready),
        u8::from(sample.redundancy_ready),
        sample
            .selected_gnss
            .map(|id| id.index() as i8)
            .unwrap_or(-1),
        sample.height_msl_m,
        sample.velocity_mps,
        sample.barometer_bias_m[0],
        sample.barometer_bias_m[1],
        sample.height_std_m,
        sample.velocity_std_mps,
        sample.barometer_bias_std_m[0],
        sample.barometer_bias_std_m[1],
        sample.consistency_score,
        w,
        x,
        y,
        z,
    )
}
