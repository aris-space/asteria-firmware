pub mod calibration;
pub mod mount;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use serde::{Deserialize, Serialize};

use crate::storage::{self, BackendLoad, KeyStorage, SaveStatus};

pub const DEFAULT_WATCHERS: usize = 4;

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[allow(dead_code)]
pub enum ConfigSource {
    Persisted,
    Runtime,
    Missing,
    Invalid,
    Unavailable,
}

#[allow(dead_code)]
#[derive(Clone, Debug)]
pub struct ConfigSnapshot<T> {
    pub source: ConfigSource,
    pub value: Option<T>,
}

impl<T> ConfigSnapshot<T> {
    pub const fn new(source: ConfigSource, value: Option<T>) -> Self {
        Self { source, value }
    }
}

pub(crate) struct RegistryEntry {
    load: fn(&dyn KeyStorage),
}

impl RegistryEntry {
    pub const fn new(load: fn(&dyn KeyStorage)) -> Self {
        Self { load }
    }

    fn load(&self, backend: &dyn KeyStorage) {
        (self.load)(backend);
    }
}

macro_rules! indexed_loaders {
    ($configs:ident, $($loader:ident => $index:expr),+ $(,)?) => {
        $(
            fn $loader(backend: &dyn $crate::storage::KeyStorage) {
                $configs[$index].load_from_backend(backend);
            }
        )+
    };
}

pub(crate) use indexed_loaders;

macro_rules! registry_entries {
    ($($loader:path),+ $(,)?) => {
        [$(RegistryEntry::new($loader)),+]
    };
}

pub(crate) use registry_entries;

pub struct Config<T: Clone, const BYTES: usize, const WATCHERS: usize = DEFAULT_WATCHERS> {
    key: &'static str,
    default: T,
    state: Watch<CriticalSectionRawMutex, ConfigSnapshot<T>, WATCHERS>,
}

#[allow(dead_code)]
impl<T: Clone, const BYTES: usize, const WATCHERS: usize> Config<T, BYTES, WATCHERS> {
    pub const fn new(key: &'static str, default: T) -> Self {
        Self {
            key,
            default,
            state: Watch::new_with(ConfigSnapshot::new(ConfigSource::Unavailable, None)),
        }
    }

    pub fn default_value(&self) -> T {
        self.default.clone()
    }

    pub fn snapshot(&self) -> ConfigSnapshot<T> {
        self.state
            .try_get()
            .expect("config snapshots are always initialized")
    }

    pub fn set_runtime(&self, value: T) {
        self.state
            .sender()
            .send(ConfigSnapshot::new(ConfigSource::Runtime, Some(value)));
    }

    fn set_snapshot(&self, snapshot: ConfigSnapshot<T>) {
        self.state.sender().send(snapshot);
    }
}

#[allow(dead_code)]
impl<T, const BYTES: usize, const WATCHERS: usize> Config<T, BYTES, WATCHERS>
where
    T: Clone + Serialize + for<'de> Deserialize<'de> + Send,
{
    pub fn load_from_backend(&self, backend: &dyn KeyStorage) {
        let mut buf = [0u8; BYTES];

        match backend.load(self.key, &mut buf) {
            BackendLoad::Loaded(len) => match postcard::from_bytes::<T>(&buf[..len]) {
                Ok(value) => {
                    self.set_snapshot(ConfigSnapshot::new(ConfigSource::Persisted, Some(value)));
                }
                Err(_) => {
                    self.set_snapshot(ConfigSnapshot::new(ConfigSource::Invalid, None));
                }
            },
            BackendLoad::Missing => {
                self.set_snapshot(ConfigSnapshot::new(ConfigSource::Missing, None));
            }
            BackendLoad::Unavailable => {
                self.set_snapshot(ConfigSnapshot::new(ConfigSource::Unavailable, None));
            }
        }
    }

    pub async fn set_and_save(&'static self, value: T) -> SaveStatus {
        self.set_runtime(value.clone());

        let snapshot = self.snapshot();
        let Some(value) = snapshot.value else {
            return SaveStatus::RuntimeOnly;
        };

        let mut buf = [0u8; BYTES];
        let len = match postcard::to_slice(&value, &mut buf) {
            Ok(bytes) => bytes.len(),
            Err(_) => return SaveStatus::RuntimeOnly,
        };

        let status = storage::save(self.key, buf, len).await;
        if matches!(status, SaveStatus::Persisted) {
            self.set_snapshot(ConfigSnapshot::new(ConfigSource::Persisted, Some(value)));
        }

        status
    }
}

pub fn load_all(backend: &dyn KeyStorage) {
    for entry in calibration::REGISTRY.iter().chain(mount::REGISTRY.iter()) {
        entry.load(backend);
    }
}
