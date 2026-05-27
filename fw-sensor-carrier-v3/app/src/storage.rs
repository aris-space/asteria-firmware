//! Persistent key/value storage on the on-board W25Q01JV (OCTOSPI1) via
//! `sequential-storage`'s map. Keys are short ASCII names ([`key`]); values
//! are postcard-encoded. The config region backs the map; the rest stays free
//! for future data logging.

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{MapConfig, MapStorage};
use serde::{Deserialize, Serialize};
use static_cell::StaticCell;

use crate::resources::flash::{BoardFlash, JedecId};

pub const CONFIG_OFFSET: u32 = 0;
pub const CONFIG_LEN: u32 = 64 * 1024;
/// Scratch size for a serialized key+value. Fits a [`KEY_LEN`]-byte key plus the
/// largest cal value with headroom.
const MAX_BYTES: usize = 128;
pub const KEY_LEN: usize = 16;

/// Fixed-size storage key: a short ASCII name, zero-padded.
pub type Key = [u8; KEY_LEN];

/// Build a [`Key`] from a short name. Panics at const-eval if it exceeds [`KEY_LEN`].
pub const fn key(name: &str) -> Key {
    let bytes = name.as_bytes();
    assert!(bytes.len() <= KEY_LEN, "storage key name exceeds 16 bytes");
    let mut out = [0u8; KEY_LEN];
    let mut i = 0;
    while i < bytes.len() {
        out[i] = bytes[i];
        i += 1;
    }
    out
}

type Map = MapStorage<Key, BoardFlash, NoCache>;

pub struct Storage {
    map: Mutex<CriticalSectionRawMutex, Map>,
}

static STORAGE: StaticCell<Storage> = StaticCell::new();

impl Storage {
    pub fn init(flash: BoardFlash) -> &'static Storage {
        let map = MapStorage::new(
            flash,
            const { MapConfig::new(CONFIG_OFFSET..CONFIG_OFFSET + CONFIG_LEN) },
            NoCache::new(),
        );
        STORAGE.init(Storage {
            map: Mutex::new(map),
        })
    }

    /// Decode the value at `key`, or `None` if it's absent or undecodable.
    pub async fn load<T: for<'de> Deserialize<'de>>(&self, key: &Key) -> Option<T> {
        let mut scratch = [0u8; MAX_BYTES];
        let mut map = self.map.lock().await;
        let bytes: &[u8] = match map.fetch_item(&mut scratch, key).await {
            Ok(found) => found?,
            Err(e) => {
                defmt::warn!("storage load failed: {:?}", defmt::Debug2Format(&e));
                return None;
            }
        };
        postcard::from_bytes(bytes).ok()
    }

    /// Persist `value` at `key`. Returns `false` if encoding or the flash
    /// write failed.
    pub async fn store<T: Serialize>(&self, key: &Key, value: &T) -> bool {
        let mut buf = [0u8; MAX_BYTES];
        let used = match postcard::to_slice(value, &mut buf) {
            Ok(b) => b.len(),
            Err(_) => return false,
        };
        let value_bytes: &[u8] = &buf[..used];
        let mut scratch = [0u8; MAX_BYTES];
        let mut map = self.map.lock().await;
        match map.store_item(&mut scratch, key, &value_bytes).await {
            Ok(()) => true,
            Err(e) => {
                defmt::warn!("storage store failed: {:?}", defmt::Debug2Format(&e));
                false
            }
        }
    }

    /// Delete a single key in place. Relies on the flash being
    /// `MultiwriteNorFlash` (the delete flips one flag bit, no erase).
    pub async fn remove(&self, key: &Key) -> bool {
        let mut scratch = [0u8; MAX_BYTES];
        let mut map = self.map.lock().await;
        match map.remove_item(&mut scratch, key).await {
            Ok(()) => true,
            Err(e) => {
                defmt::warn!("storage remove failed: {:?}", defmt::Debug2Format(&e));
                false
            }
        }
    }

    /// Erase the whole region (factory-reset stored config). Also recovers a
    /// region left in an incompatible layout by older firmware.
    pub async fn erase(&self) -> bool {
        let mut map = self.map.lock().await;
        match map.erase_all().await {
            Ok(()) => true,
            Err(e) => {
                defmt::warn!("storage erase failed: {:?}", defmt::Debug2Format(&e));
                false
            }
        }
    }

    /// Read the flash chip's 3-byte JEDEC id (for diagnostics).
    pub async fn read_jedec_id(&self) -> JedecId {
        let mut map = self.map.lock().await;
        map.flash().read_jedec_id()
    }

    /// Read flash status register 1 (for diagnostics).
    pub async fn status(&self) -> u8 {
        let mut map = self.map.lock().await;
        map.flash().status()
    }
}
