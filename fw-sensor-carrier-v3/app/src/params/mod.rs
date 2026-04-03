pub mod calibration;
pub mod mount;

use core::cell::RefCell;
use core::fmt::Write as _;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use littlefs2::path::PathBuf;
use serde::{Deserialize, Serialize};

use crate::tasks::storage;

pub const DEFAULT_WATCHERS: usize = 4;

pub(crate) struct RegistryEntry {
    load_from_fs: fn(&storage::Fs),
    save_to_fs: fn(&storage::Fs),
}

impl RegistryEntry {
    pub const fn new(load_from_fs: fn(&storage::Fs), save_to_fs: fn(&storage::Fs)) -> Self {
        Self {
            load_from_fs,
            save_to_fs,
        }
    }

    fn load_from_fs(&self, fs: &storage::Fs) {
        (self.load_from_fs)(fs);
    }

    fn save_to_fs(&self, fs: &storage::Fs) {
        (self.save_to_fs)(fs);
    }
}

pub struct Config<T: Clone, const BYTES: usize, const WATCHERS: usize = DEFAULT_WATCHERS> {
    key: &'static str,
    value: Watch<CriticalSectionRawMutex, T, WATCHERS>,
}

impl<T: Clone, const BYTES: usize, const WATCHERS: usize> Config<T, BYTES, WATCHERS> {
    pub const fn new(key: &'static str, default: T) -> Self {
        Self {
            key,
            value: Watch::new_with(default),
        }
    }

    pub fn get(&self) -> T {
        self.value
            .try_get()
            .expect("config values are always initialized")
    }

    #[allow(dead_code)]
    pub fn set(&self, value: T) {
        self.value.sender().send(value);
    }

    #[allow(dead_code)]
    pub fn update<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        let f = RefCell::new(Some(f));
        let result = RefCell::new(None);

        self.value.sender().send_modify(|slot: &mut Option<T>| {
            let value = slot.as_mut().expect("config values are always initialized");
            *result.borrow_mut() = Some(
                f.borrow_mut().take().expect("update closure called once")(value),
            );
        });

        result
            .into_inner()
            .expect("update closure must produce a value")
    }

    pub fn receiver(
        &'static self,
    ) -> Option<Receiver<'static, CriticalSectionRawMutex, T, WATCHERS>> {
        self.value.receiver()
    }

    fn path(&self) -> Option<PathBuf> {
        let mut path = heapless::String::<64>::new();
        write!(path, "/params/{}.bin", self.key).ok()?;
        PathBuf::try_from(path.as_bytes()).ok()
    }
}

impl<T, const BYTES: usize, const WATCHERS: usize> Config<T, BYTES, WATCHERS>
where
    T: Clone + Serialize + for<'de> Deserialize<'de> + Send,
{
    fn restore_from_bytes(&self, bytes: &[u8]) {
        if let Ok(value) = postcard::from_bytes::<T>(bytes) {
            self.value.sender().send(value);
        }
    }

    pub fn load_from_fs(&self, fs: &storage::Fs) {
        let Some(path) = self.path() else {
            return;
        };
        let mut buf = [0u8; BYTES];
        let Ok(len) = fs.open_file_and_then(&path, |file| file.read(&mut buf)) else {
            return;
        };
        self.restore_from_bytes(&buf[..len]);
    }

    pub fn save_to_fs(&self, fs: &storage::Fs) {
        let Some(path) = self.path() else {
            return;
        };
        let data = self.get();
        let mut buf = [0u8; BYTES];
        let Ok(bytes) = postcard::to_slice(&data, &mut buf) else {
            return;
        };
        let len = bytes.len();

        let _ = fs.create_dir_all(littlefs2::path!("/params"));
        let _ = fs.open_file_with_options_and_then(
            |options| options.create(true).truncate(true),
            &path,
            |file| file.write(&buf[..len]),
        );
    }

    #[allow(dead_code)]
    pub async fn load(&'static self) {
        if !storage::is_available() {
            return;
        }
        storage::FS.call(move |fs| self.load_from_fs(fs)).await;
    }

    #[allow(dead_code)]
    pub async fn save(&'static self) {
        if !storage::is_available() {
            return;
        }
        storage::FS.call(move |fs| self.save_to_fs(fs)).await;
    }

    #[allow(dead_code)]
    pub async fn set_and_save(&'static self, value: T) {
        self.set(value);
        self.save().await;
    }

    #[allow(dead_code)]
    pub async fn update_and_save<R>(&'static self, f: impl FnOnce(&mut T) -> R) -> R {
        let result = self.update(f);
        self.save().await;
        result
    }
}

pub fn load_all(fs: &storage::Fs) {
    for entry in calibration::REGISTRY.iter().chain(mount::REGISTRY.iter()) {
        entry.load_from_fs(fs);
    }
}

#[allow(dead_code)]
pub async fn save_all() {
    if !storage::is_available() {
        return;
    }
    storage::FS
        .call(|fs| {
            for entry in calibration::REGISTRY.iter().chain(mount::REGISTRY.iter()) {
                entry.save_to_fs(fs);
            }
        })
        .await;
}
