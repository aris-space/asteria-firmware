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
pub(crate) enum BackendLoad {
    Loaded(usize),
    Missing,
    Unavailable,
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
pub(crate) trait KeyStorage {
    fn load(&self, key: &str, out: &mut [u8]) -> BackendLoad;
    fn save(&self, key: &str, data: &[u8]) -> SaveStatus;
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
pub(crate) trait FileStorage {
    fn read_dir(&self, path: &str, visitor: &mut dyn FnMut(&str)) -> bool;
    fn create_dir(&self, path: &str) -> bool;
    fn write_file(&self, path: &str, data: &[u8]) -> bool;
    fn append_file(&self, path: &str, data: &[u8]) -> bool;
}

pub(crate) async fn save<const N: usize>(
    key: &'static str,
    data: [u8; N],
    len: usize,
) -> SaveStatus {
    backend::save(key, data, len).await
}

pub(crate) use backend::task;

struct UnavailableStorage;

impl KeyStorage for UnavailableStorage {
    fn load(&self, _key: &str, _out: &mut [u8]) -> BackendLoad {
        BackendLoad::Unavailable
    }

    fn save(&self, _key: &str, _data: &[u8]) -> SaveStatus {
        SaveStatus::RuntimeOnly
    }
}

impl FileStorage for UnavailableStorage {
    fn read_dir(&self, _path: &str, _visitor: &mut dyn FnMut(&str)) -> bool {
        false
    }

    fn create_dir(&self, _path: &str) -> bool {
        false
    }

    fn write_file(&self, _path: &str, _data: &[u8]) -> bool {
        false
    }

    fn append_file(&self, _path: &str, _data: &[u8]) -> bool {
        false
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
