//! `flash info`, `flash list`, `flash clear` and `flash erase`.

use core::str::SplitAsciiWhitespace;

use super::io::{ConsoleIo, say, sayf};
use crate::calibration::{self, Calibrations, Correction, baro, dht, gnss, imu, mag};
use crate::resources::flash;
use crate::sensors::SensorId;
use crate::storage::{self, Storage};

pub async fn command(
    io: &mut ConsoleIo<'_>,
    args: &mut SplitAsciiWhitespace<'_>,
    storage: &Storage,
) {
    match args.next() {
        Some("info") => flash_info(io, storage).await,
        Some("list") => flash_list(io).await,
        Some("clear") => {
            match args.next() {
                Some(name) if args.next() == Some("--yes") => flash_clear(io, storage, name).await,
                Some(_) => say(
                    io,
                    paint!(
                        yellow,
                        "refusing: this clears a stored key. re-run as 'flash clear <key> --yes'\n"
                    ),
                )
                .await,
                None => say(io, paint!(red, "usage: flash clear <key> --yes\n")).await,
            }
        }
        Some("erase") if args.next() == Some("--yes") => flash_erase(io, storage).await,
        Some("erase") => {
            say(
                io,
                paint!(
                    yellow,
                    "refusing: this wipes ALL stored config. re-run as 'flash erase --yes'\n"
                ),
            )
            .await
        }
        _ => {
            say(
                io,
                paint!(red, "usage: flash <list|clear <key> --yes|erase --yes>\n"),
            )
            .await
        }
    }
}

async fn flash_info(io: &mut ConsoleIo<'_>, storage: &Storage) {
    let id = storage.read_jedec_id().await;
    let status = storage.status().await;
    let detected = if id.manufacturer == flash::WINBOND_MANUFACTURER_ID
        && id.memory_type == flash::W25Q_IM_MEMORY_TYPE
        && id.capacity == flash::W25Q01JV_CAPACITY
    {
        "W25Q01JV"
    } else if id.manufacturer == flash::WINBOND_MANUFACTURER_ID {
        "Winbond, unexpected JEDEC id"
    } else {
        "UNKNOWN (check wiring/power)"
    };
    sayf(
        io,
        format_args!(
            "jedec id:   {:02x} {:02x} {:02x}\ndetected:   {detected}\nstatus reg: {:#04x} (wip={})\ncapacity:   {} MiB, config region {} KiB\n",
            id.manufacturer,
            id.memory_type,
            id.capacity,
            status,
            status & 1,
            flash::CAPACITY / (1024 * 1024),
            storage::CONFIG_LEN / 1024,
        ),
    )
    .await;
}

async fn flash_list(io: &mut ConsoleIo<'_>) {
    list_keys(io, &imu::CAL).await;
    list_keys(io, &mag::CAL).await;
    list_keys(io, &gnss::CAL).await;
    list_keys(io, &baro::CAL).await;
    list_keys(io, &dht::CAL).await;
}

async fn list_keys<Id: SensorId, C: Correction, const N: usize>(
    io: &mut ConsoleIo<'_>,
    cals: &Calibrations<Id, C, N>,
) {
    for id in cals.ids() {
        let key = calibration::key(id);
        let len = key.iter().position(|&b| b == 0).unwrap_or(key.len());
        let name = core::str::from_utf8(&key[..len]).unwrap_or("?");
        sayf(io, format_args!("{name}\n")).await;
    }
}

async fn flash_clear(io: &mut ConsoleIo<'_>, storage: &Storage, name: &str) {
    if name.len() > storage::KEY_LEN {
        sayf(
            io,
            format_args!(
                paint!(red, "key name too long (max {})\n"),
                storage::KEY_LEN
            ),
        )
        .await;
    } else {
        match storage.remove(&storage::key(name)).await {
            Ok(true) => {
                sayf(
                    io,
                    format_args!(paint!(green, "cleared {}; reset to apply.\n"), name),
                )
                .await
            }
            Ok(false) => {
                sayf(
                    io,
                    format_args!(paint!(yellow, "{} is not stored; nothing cleared.\n"), name),
                )
                .await
            }
            Err(()) => say(io, paint!(red, "flash error\n")).await,
        }
    }
}

async fn flash_erase(io: &mut ConsoleIo<'_>, storage: &Storage) {
    let msg = if storage.erase().await {
        paint!(green, "erased flash; reset to apply.\n")
    } else {
        paint!(red, "flash error\n")
    };
    say(io, msg).await;
}
