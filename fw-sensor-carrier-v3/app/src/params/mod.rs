pub mod calibration;
pub mod mount;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use serde::{Deserialize, Serialize};

use crate::tasks::storage;

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

#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
#[allow(dead_code)]
pub enum SaveStatus {
    Persisted,
    RuntimeOnly,
}

#[allow(dead_code)]
pub enum BackendLoad {
    Loaded(usize),
    Missing,
    Unavailable,
}

pub trait ConfigBackend {
    fn load(&self, key: &str, out: &mut [u8]) -> BackendLoad;
}

pub(crate) struct RegistryEntry {
    load: fn(&dyn ConfigBackend),
}

impl RegistryEntry {
    pub const fn new(load: fn(&dyn ConfigBackend)) -> Self {
        Self { load }
    }

    fn load(&self, backend: &dyn ConfigBackend) {
        (self.load)(backend);
    }
}

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

    pub fn effective(&self) -> Option<T> {
        self.snapshot().value
    }

    pub fn effective_or_default(&self) -> T {
        self.effective().unwrap_or_else(|| self.default_value())
    }

    pub fn receiver(
        &'static self,
    ) -> Option<Receiver<'static, CriticalSectionRawMutex, ConfigSnapshot<T>, WATCHERS>> {
        self.state.receiver()
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
    pub fn load_from_backend(&self, backend: &dyn ConfigBackend) {
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
        self.set_runtime(value);

        let snapshot = self.snapshot();
        let Some(value) = snapshot.value else {
            return SaveStatus::RuntimeOnly;
        };

        let mut buf = [0u8; BYTES];
        let Ok(bytes) = postcard::to_slice(&value, &mut buf) else {
            return SaveStatus::RuntimeOnly;
        };

        storage::save(self.key, bytes).await
    }
}

pub fn load_all(backend: &dyn ConfigBackend) {
    for entry in calibration::REGISTRY.iter().chain(mount::REGISTRY.iter()) {
        entry.load(backend);
    }
}
