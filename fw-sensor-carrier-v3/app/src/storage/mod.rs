mod backend;
#[cfg(feature = "storage")]
mod session;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

pub(crate) static CONFIG_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub(crate) const STORAGE_KEY_CAPACITY: usize = 32;

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct StorageKey {
    len: u8,
    bytes: [u8; STORAGE_KEY_CAPACITY],
}

impl StorageKey {
    pub const fn new(key: &'static str) -> Self {
        let key_bytes = key.as_bytes();
        assert!(
            key_bytes.len() <= STORAGE_KEY_CAPACITY,
            "storage key too long"
        );

        let mut bytes = [0; STORAGE_KEY_CAPACITY];
        let mut i = 0;
        while i < key_bytes.len() {
            bytes[i] = key_bytes[i];
            i += 1;
        }

        Self {
            len: key_bytes.len() as u8,
            bytes,
        }
    }

    pub fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len as usize]).expect("storage keys are utf-8")
    }
}

impl core::fmt::Debug for StorageKey {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(self.as_str())
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub(crate) enum SaveStatus {
    Persisted,
    RuntimeOnly,
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct StorageUnavailable;

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
pub(crate) type StorageResult<T = ()> = Result<T, StorageUnavailable>;

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub(crate) enum KeyRead {
    Found(usize),
    Missing,
    Unavailable,
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
pub(crate) trait KeyStorage {
    async fn read_key(&mut self, key: StorageKey, out: &mut [u8]) -> KeyRead;
    async fn write_key(&mut self, key: StorageKey, data: &[u8]) -> SaveStatus;
}

pub(crate) use backend::run;

struct UnavailableStorage;

impl KeyStorage for UnavailableStorage {
    async fn read_key(&mut self, _key: StorageKey, _out: &mut [u8]) -> KeyRead {
        KeyRead::Unavailable
    }

    async fn write_key(&mut self, _key: StorageKey, _data: &[u8]) -> SaveStatus {
        SaveStatus::RuntimeOnly
    }
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum LogWriteStatus {
    Stored,
    Full,
    Unavailable,
}
