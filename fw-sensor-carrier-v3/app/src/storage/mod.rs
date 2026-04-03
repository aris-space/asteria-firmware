mod backend;
#[cfg(feature = "storage")]
mod session;

#[cfg(feature = "storage")]
use core::sync::atomic::AtomicU32;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

pub(crate) static CONFIG_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
static AVAILABLE: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "storage")]
static SESSION_INDEX: AtomicU32 = AtomicU32::new(u32::MAX);

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
pub(crate) enum KeyRead {
    Found(usize),
    Missing,
    Unavailable,
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
pub(crate) trait KeyStorage {
    fn read_key(&self, key: &str, out: &mut [u8]) -> KeyRead;
    fn write_key(&self, key: &str, data: &[u8]) -> StorageResult;
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum FileWriteMode {
    Overwrite,
    Append,
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
pub(crate) trait FileStorage {
    fn visit_dir(&self, path: &str, visitor: &mut dyn FnMut(&str)) -> StorageResult;
    fn ensure_dir(&self, path: &str) -> StorageResult;
    fn write_file(&self, path: &str, data: &[u8], mode: FileWriteMode) -> StorageResult;
}

pub(crate) async fn persist_key<const N: usize>(
    key: &'static str,
    data: [u8; N],
    len: usize,
) -> SaveStatus {
    backend::persist_key(key, data, len).await
}

pub(crate) use backend::task;

struct UnavailableStorage;

impl KeyStorage for UnavailableStorage {
    fn read_key(&self, _key: &str, _out: &mut [u8]) -> KeyRead {
        KeyRead::Unavailable
    }

    fn write_key(&self, _key: &str, _data: &[u8]) -> StorageResult {
        Err(StorageUnavailable)
    }
}

impl FileStorage for UnavailableStorage {
    fn visit_dir(&self, _path: &str, _visitor: &mut dyn FnMut(&str)) -> StorageResult {
        Err(StorageUnavailable)
    }

    fn ensure_dir(&self, _path: &str) -> StorageResult {
        Err(StorageUnavailable)
    }

    fn write_file(&self, _path: &str, _data: &[u8], _mode: FileWriteMode) -> StorageResult {
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

#[cfg(feature = "storage")]
fn session_index() -> Option<u32> {
    match SESSION_INDEX.load(Ordering::Relaxed) {
        u32::MAX => None,
        index => Some(index),
    }
}

#[cfg(feature = "storage")]
fn set_session_index(index: u32) {
    SESSION_INDEX.store(index, Ordering::Relaxed);
}
