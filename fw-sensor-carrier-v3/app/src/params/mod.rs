pub mod calibration;
pub mod mount;

use core::cell::RefCell;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use serde::{Deserialize, Serialize};

use crate::storage::{KeyRead, KeyStorage, SaveStatus, StorageKey};

pub struct Config<T: Clone, const BYTES: usize> {
    key: StorageKey,
    state: Mutex<CriticalSectionRawMutex, RefCell<Option<T>>>,
}

impl<T: Clone, const BYTES: usize> Config<T, BYTES> {
    pub const fn new(key: &'static str) -> Self {
        Self {
            key: StorageKey::new(key),
            state: Mutex::new(RefCell::new(None)),
        }
    }

    pub fn value(&self) -> Option<T> {
        self.state.lock(|state| state.borrow().clone())
    }

    #[allow(dead_code)]
    pub fn set_runtime(&self, value: T) {
        self.state.lock(|state| {
            *state.borrow_mut() = Some(value);
        });
    }

    fn set_value(&self, value: Option<T>) {
        self.state.lock(|state| {
            *state.borrow_mut() = value;
        });
    }
}

impl<T: Clone + Default, const BYTES: usize> Config<T, BYTES> {
    pub fn value_or_default(&self) -> T {
        self.value().unwrap_or_default()
    }
}

impl<T, const BYTES: usize> Config<T, BYTES>
where
    T: Clone + Serialize + for<'de> Deserialize<'de> + Send,
{
    pub async fn load_from_backend(&self, backend: &mut impl KeyStorage) {
        let mut buf = [0u8; BYTES];

        match backend.read_key(self.key, &mut buf).await {
            KeyRead::Found(len) => {
                self.set_value(postcard::from_bytes::<T>(&buf[..len]).ok());
            }
            KeyRead::Missing | KeyRead::Unavailable => {
                self.set_value(None);
            }
        }
    }

    #[allow(dead_code)]
    pub async fn set_and_save(&self, backend: &mut impl KeyStorage, value: T) -> SaveStatus {
        let mut buf = [0u8; BYTES];
        let len = match postcard::to_slice(&value, &mut buf) {
            Ok(bytes) => bytes.len(),
            Err(_) => return SaveStatus::RuntimeOnly,
        };

        self.set_runtime(value);
        backend.write_key(self.key, &buf[..len]).await
    }
}

pub(super) async fn load_all_configs<T, const BYTES: usize, const N: usize>(
    backend: &mut impl KeyStorage,
    configs: &[Config<T, BYTES>; N],
) where
    T: Clone + Serialize + for<'de> Deserialize<'de> + Send,
{
    for config in configs {
        config.load_from_backend(backend).await;
    }
}

pub async fn load_all(backend: &mut impl KeyStorage) {
    calibration::load_all(backend).await;
    mount::load_all(backend).await;
}
