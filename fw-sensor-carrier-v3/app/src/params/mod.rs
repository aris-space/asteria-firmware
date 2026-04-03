pub mod calibration;
pub mod mount;

use core::cell::RefCell;
use core::fmt::Write as _;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use littlefs2::path::PathBuf;
use serde::{Deserialize, Serialize};

use crate::tasks::storage;

pub struct Table<T> {
    name: &'static str,
    data: Mutex<CriticalSectionRawMutex, RefCell<T>>,
}

impl<T: Clone> Table<T> {
    pub const fn new(name: &'static str, default: T) -> Self {
        Self {
            name,
            data: Mutex::new(RefCell::new(default)),
        }
    }

    pub fn snapshot(&self) -> T {
        self.data.lock(|cell| cell.borrow().clone())
    }

    pub fn update<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        self.data.lock(|cell| f(&mut cell.borrow_mut()))
    }
}

impl<T: Clone + Serialize + for<'de> Deserialize<'de>> Table<T> {
    fn restore_from_bytes(&self, bytes: &[u8]) {
        if let Ok(data) = postcard::from_bytes::<T>(bytes) {
            self.update(|table| *table = data);
        }
    }

    fn param_path(&self) -> Option<PathBuf> {
        let mut s = heapless::String::<64>::new();
        write!(s, "/params/{}.bin", self.name).ok()?;
        PathBuf::try_from(s.as_bytes()).ok()
    }

    pub fn load_from_fs<S: littlefs2::driver::Storage>(
        &self,
        fs: &littlefs2::fs::Filesystem<'_, S>,
    ) {
        let Some(path) = self.param_path() else {
            return;
        };
        let mut buf = [0u8; 256];
        let mut len = 0usize;
        let Ok(()) = fs.open_file_and_then(&path, |file| {
            len = file.read(&mut buf)?;
            Ok(())
        }) else {
            return;
        };
        self.restore_from_bytes(&buf[..len]);
    }

    #[allow(dead_code)]
    pub async fn save(&'static self) {
        let Some(path) = self.param_path() else {
            return;
        };
        let data = self.snapshot();
        let mut buf = [0u8; 256];
        let Ok(bytes) = postcard::to_slice(&data, &mut buf) else {
            return;
        };
        let len = bytes.len();

        storage::FS
            .call(move |fs| {
                let _ = fs.create_dir_all(littlefs2::path!("/params"));
                let _ = fs.open_file_with_options_and_then(
                    |o| o.create(true).truncate(true),
                    &path,
                    |file| file.write(&buf[..len]),
                );
            })
            .await;
    }

    #[allow(dead_code)]
    pub async fn load(&'static self) {
        let Some(path) = self.param_path() else {
            return;
        };

        let result: Option<([u8; 256], usize)> = storage::FS
            .call(move |fs| -> Option<([u8; 256], usize)> {
                let mut buf = [0u8; 256];
                let len = fs
                    .open_file_and_then(&path, |file| file.read(&mut buf))
                    .ok()?;
                Some((buf, len))
            })
            .await;

        if let Some((buf, len)) = result
            && let Ok(data) = postcard::from_bytes::<T>(&buf[..len])
        {
            self.update(|table| *table = data);
        }
    }
}

pub fn load_all<S: littlefs2::driver::Storage>(fs: &littlefs2::fs::Filesystem<'_, S>) {
    for table in &calibration::IMU_CAL {
        table.load_from_fs(fs);
    }
}
