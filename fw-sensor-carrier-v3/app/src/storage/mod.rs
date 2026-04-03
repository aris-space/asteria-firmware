mod backend;
#[cfg(feature = "storage")]
mod session;

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

pub(crate) static CONFIG_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
static AVAILABLE: AtomicBool = AtomicBool::new(false);

pub(crate) type StorageKey = &'static str;

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
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
    async fn write_key(&mut self, key: StorageKey, data: &[u8]) -> StorageResult;
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum LogWriteStatus {
    Stored,
    Full,
    Unavailable,
}

pub(crate) async fn persist_key<const N: usize>(
    key: StorageKey,
    data: [u8; N],
    len: usize,
) -> SaveStatus {
    backend::persist_key(key, data, len).await
}

pub(crate) use backend::task;

struct UnavailableStorage;

impl KeyStorage for UnavailableStorage {
    async fn read_key(&mut self, _key: StorageKey, _out: &mut [u8]) -> KeyRead {
        KeyRead::Unavailable
    }

    async fn write_key(&mut self, _key: StorageKey, _data: &[u8]) -> StorageResult {
        Err(StorageUnavailable)
    }
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
fn is_available() -> bool {
    AVAILABLE.load(Ordering::Relaxed)
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
fn set_available(available: bool) {
    AVAILABLE.store(available, Ordering::Relaxed);
}
