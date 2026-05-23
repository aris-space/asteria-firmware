//! Parameter storage for the sensor carrier.
//!
//! Backed by the on-board W25Q256JV (SPI2) via `sequential-storage`.
//! Registry entries land here as later phases migrate each calibration
//! source from compile-time consts to runtime-tunable params.

use defmt::{info, warn};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use firmware_params::{Key, LoadStatus, ParamAccess, ParamEntry, ParamStorage, Registry, key_str};
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};
use static_cell::StaticCell;

use crate::calibration::{imu, mag};
use crate::commands;
use crate::resources::flash::BoardFlash;

/// Reserve the first 64 KiB of the chip (sixteen 4 KiB sectors) for the
/// param map; the rest stays free for future data-logging use.
const PARAMS_FLASH_OFFSET: u32 = 0;
const PARAMS_FLASH_LEN: u32 = 64 * 1024;

pub type Access = ParamAccess<CriticalSectionRawMutex, &'static mut BoardFlash, NoCache>;

pub static REGISTRY: Registry = Registry::new(&[
    ParamEntry::persistent(&mag::MAG_0_PARAM),
    ParamEntry::persistent(&mag::MAG_1_PARAM),
    ParamEntry::persistent(&imu::IMU_0_PARAM),
    ParamEntry::persistent(&imu::IMU_1_PARAM),
    ParamEntry::volatile(&commands::RESET),
    ParamEntry::volatile(&commands::RUN_MAG_CAL),
    ParamEntry::volatile(&commands::RUN_IMU_CAL),
]);

static ACCESS: StaticCell<Access> = StaticCell::new();

pub fn init(flash: &'static mut BoardFlash) -> &'static Access {
    let map = MapStorage::<Key, _, _>::new(
        flash,
        const { MapConfig::new(PARAMS_FLASH_OFFSET..PARAMS_FLASH_OFFSET + PARAMS_FLASH_LEN) },
        NoCache::new(),
    );
    ACCESS.init(ParamAccess::new(&REGISTRY, map))
}

pub async fn load_all(access: &Access) {
    for entry in REGISTRY
        .entries()
        .filter(|e| e.storage() == ParamStorage::Persistent)
    {
        match access.load(entry).await {
            Ok(LoadStatus::Loaded) => info!("param {}: loaded", key_str(entry.key())),
            Ok(LoadStatus::Missing) => info!("param {}: missing", key_str(entry.key())),
            Ok(LoadStatus::BadDecode(_)) => warn!("param {}: bad decode", key_str(entry.key())),
            Err(_) => warn!("param {}: flash io error", key_str(entry.key())),
        }
    }
}
