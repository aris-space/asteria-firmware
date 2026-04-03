#[cfg(feature = "storage")]
use core::fmt::Write as _;
#[cfg(feature = "storage")]
use core::sync::atomic::AtomicU32;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt_brtt::DefmtConsumer;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::resources::flash::BoardFlash;

pub static CONFIG_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
static AVAILABLE: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "storage")]
static SESSION_INDEX: AtomicU32 = AtomicU32::new(u32::MAX);

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum SaveStatus {
    Persisted,
    RuntimeOnly,
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
pub enum BackendLoad {
    Loaded(usize),
    Missing,
    Unavailable,
}

pub trait KeyStorage {
    fn load(&self, key: &str, out: &mut [u8]) -> BackendLoad;
    fn save(&self, key: &str, data: &[u8]) -> SaveStatus;
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
pub trait FileStorage {
    fn read_dir(&self, path: &str, visitor: &mut dyn FnMut(&str)) -> bool;
    fn create_dir(&self, path: &str) -> bool;
    fn write_file(&self, path: &str, data: &[u8]) -> bool;
    fn append_file(&self, path: &str, data: &[u8]) -> bool;
}

#[cfg_attr(not(feature = "storage"), allow(dead_code))]
pub fn is_available() -> bool {
    AVAILABLE.load(Ordering::Relaxed)
}

pub async fn save<const N: usize>(key: &'static str, data: [u8; N], len: usize) -> SaveStatus {
    backend::save(key, data, len).await
}

pub use backend::task;

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

#[cfg(feature = "storage")]
fn session_idx() -> Option<u32> {
    match SESSION_INDEX.load(Ordering::Relaxed) {
        u32::MAX => None,
        index => Some(index),
    }
}

#[cfg(feature = "storage")]
fn session_dir_path(index: u32) -> Option<heapless::String<32>> {
    let mut path = heapless::String::<32>::new();
    write!(path, "/log_{index}").ok()?;
    Some(path)
}

#[cfg(feature = "storage")]
fn session_file_path(index: u32, filename: &str) -> Option<heapless::String<64>> {
    let dir = session_dir_path(index)?;
    let mut path = heapless::String::<64>::new();
    write!(path, "{dir}/{filename}").ok()?;
    Some(path)
}

#[cfg(feature = "storage")]
fn active_session_file_path(filename: &str) -> Option<heapless::String<64>> {
    session_file_path(session_idx()?, filename)
}

#[cfg(feature = "storage")]
fn config_file_path(key: &str) -> Option<heapless::String<64>> {
    let mut path = heapless::String::<64>::new();
    write!(path, "/params/{key}.bin").ok()?;
    Some(path)
}

#[cfg(feature = "storage")]
fn next_session_index(files: &dyn FileStorage) -> u32 {
    let mut max_index: Option<u32> = None;
    let mut visit = |name: &str| {
        if let Some(suffix) = name.strip_prefix("log_")
            && let Ok(index) = suffix.parse::<u32>()
        {
            max_index = Some(max_index.map_or(index, |current| current.max(index)));
        }
    };
    let _ = files.read_dir("/", &mut visit);
    max_index.map_or(0, |index| index + 1)
}

#[cfg(feature = "storage")]
fn write_build_info(files: &dyn FileStorage, session: u32) {
    use crate::built;

    let Some(path) = session_file_path(session, "build_info.txt") else {
        return;
    };

    let mut buf = heapless::String::<512>::new();
    let _ = write!(
        buf,
        "pkg={}\nprofile={}\ntarget={}\ngit={}\ndirty={}\nfeatures={}\n",
        built::PKG_NAME,
        built::PROFILE,
        built::TARGET,
        built::GIT_COMMIT_HASH_SHORT.unwrap_or("none"),
        match built::GIT_DIRTY {
            Some(true) => "true",
            Some(false) => "false",
            None => "none",
        },
        built::FEATURES_LOWERCASE_STR,
    );

    let _ = files.write_file(&path, buf.as_bytes());
}

#[cfg(feature = "storage")]
fn prepare_session(files: &dyn FileStorage) -> Option<u32> {
    let session = next_session_index(files);
    let dir = session_dir_path(session)?;
    files.create_dir(&dir).then_some(())?;
    write_build_info(files, session);
    defmt::info!("storage: session {}", dir.as_str());
    Some(session)
}

#[cfg(feature = "storage")]
const STAGING_SIZE: usize = 512;

#[cfg(feature = "storage")]
struct DefmtStaging {
    buf: [u8; STAGING_SIZE],
    len: usize,
}

#[cfg(feature = "storage")]
impl DefmtStaging {
    fn new() -> Self {
        Self {
            buf: [0u8; STAGING_SIZE],
            len: 0,
        }
    }

    fn stage(&mut self, files: &dyn FileStorage, path: &str, data: &[u8]) {
        if self.len + data.len() > STAGING_SIZE {
            self.flush(files, path);
        }
        let count = data.len().min(STAGING_SIZE - self.len);
        self.buf[self.len..self.len + count].copy_from_slice(&data[..count]);
        self.len += count;
    }

    fn flush(&mut self, files: &dyn FileStorage, path: &str) {
        if self.len == 0 {
            return;
        }

        let _ = files.append_file(path, &self.buf[..self.len]);
        self.len = 0;
    }

    fn drain(&mut self, files: &dyn FileStorage, path: &str, consumer: &mut DefmtConsumer) {
        while let Ok(grant) = consumer.read() {
            let data = grant.buf();
            let len = data.len();
            self.stage(files, path, data);
            grant.release(len);
        }
        self.flush(files, path);
    }
}

#[cfg(feature = "storage")]
mod backend {
    use super::*;

    use embassy_futures::select::{Either, select};
    use embassy_sync::rpc_service::RpcService;
    use generic_array::typenum::{U1, U256};
    use littlefs2::fs::{Allocation, Filesystem};
    use littlefs2::io::Error;
    use littlefs2::path;
    use littlefs2::path::PathBuf;
    use static_cell::StaticCell;
    use w25q256jv::LittlefsAdapter;

    use crate::resources::flash::{FixedHighPin, FlashDevice};

    type Adapter = LittlefsAdapter<'static, FlashDevice, FixedHighPin, FixedHighPin, U256, U1>;
    type Fs = Filesystem<'static, Adapter>;

    static ADAPTER: StaticCell<Adapter> = StaticCell::new();
    static ALLOC: StaticCell<Allocation<Adapter>> = StaticCell::new();
    static FS: RpcService<CriticalSectionRawMutex, Fs, 512> = RpcService::new();

    struct MountedStorage<'a> {
        fs: &'a Fs,
    }

    impl MountedStorage<'_> {
        fn path(path: &str) -> Option<PathBuf> {
            PathBuf::try_from(path.as_bytes()).ok()
        }
    }

    impl KeyStorage for MountedStorage<'_> {
        fn load(&self, key: &str, out: &mut [u8]) -> BackendLoad {
            let Some(path) = super::config_file_path(key).and_then(|path| Self::path(&path)) else {
                return BackendLoad::Unavailable;
            };

            match self.fs.open_file_and_then(&path, |file| file.read(out)) {
                Ok(len) => BackendLoad::Loaded(len),
                Err(Error::NO_SUCH_ENTRY) => BackendLoad::Missing,
                Err(_) => BackendLoad::Unavailable,
            }
        }

        fn save(&self, key: &str, data: &[u8]) -> SaveStatus {
            let Some(path) = super::config_file_path(key).and_then(|path| Self::path(&path)) else {
                return SaveStatus::RuntimeOnly;
            };

            let _ = self.fs.create_dir_all(path!("/params"));
            match self.fs.open_file_with_options_and_then(
                |options| options.create(true).truncate(true),
                &path,
                |file| file.write(data),
            ) {
                Ok(_) => SaveStatus::Persisted,
                Err(_) => SaveStatus::RuntimeOnly,
            }
        }
    }

    impl FileStorage for MountedStorage<'_> {
        fn read_dir(&self, path: &str, visitor: &mut dyn FnMut(&str)) -> bool {
            let Some(path) = Self::path(path) else {
                return false;
            };

            self.fs
                .read_dir_and_then(&path, |dir| {
                    for entry in dir {
                        let entry = entry?;
                        let name = entry.file_name().as_str_ref_with_trailing_nul();
                        visitor(name.trim_end_matches('\0'));
                    }
                    Ok(())
                })
                .is_ok()
        }

        fn create_dir(&self, path: &str) -> bool {
            let Some(path) = Self::path(path) else {
                return false;
            };
            self.fs.create_dir(&path).is_ok()
        }

        fn write_file(&self, path: &str, data: &[u8]) -> bool {
            let Some(path) = Self::path(path) else {
                return false;
            };

            self.fs
                .open_file_with_options_and_then(
                    |options| options.create(true).truncate(true),
                    &path,
                    |file| file.write(data),
                )
                .is_ok()
        }

        fn append_file(&self, path: &str, data: &[u8]) -> bool {
            let Some(path) = Self::path(path) else {
                return false;
            };

            self.fs
                .open_file_with_options_and_then(
                    |options| options.append(true).create(true),
                    &path,
                    |file| file.write(data),
                )
                .is_ok()
        }
    }

    pub async fn save<const N: usize>(key: &'static str, data: [u8; N], len: usize) -> SaveStatus {
        if !super::is_available() || len > N {
            return SaveStatus::RuntimeOnly;
        }

        FS.call(move |fs| MountedStorage { fs }.save(key, &data[..len]))
            .await
    }

    #[embassy_executor::task]
    pub async fn task(flash: &'static mut BoardFlash, mut consumer: DefmtConsumer) -> ! {
        let adapter = ADAPTER.init(LittlefsAdapter::<_, _, _, U256, U1>::new(flash));
        let alloc = ALLOC.init(Filesystem::allocate());

        if Filesystem::mount(alloc, adapter).is_err() {
            defmt::info!("storage: mount failed, formatting");
            let _ = Filesystem::format(adapter);
        }

        let Ok(mut fs) = Filesystem::mount(alloc, adapter) else {
            defmt::warn!("storage: unavailable, continuing with in-memory defaults");
            crate::params::load_all(&super::UnavailableStorage);
            CONFIG_READY.signal(());

            loop {
                let grant = consumer.wait_for_log().await;
                let len = grant.buf().len();
                grant.release(len);
            }
        };

        AVAILABLE.store(true, Ordering::Relaxed);
        defmt::info!("storage: backend mounted");

        {
            let mounted = MountedStorage { fs: &fs };
            crate::params::load_all(&mounted);

            if let Some(index) = super::prepare_session(&mounted) {
                SESSION_INDEX.store(index, Ordering::Relaxed);
            }
        }

        CONFIG_READY.signal(());
        defmt::info!("storage: config initialized");

        let defmt_path = super::active_session_file_path("defmt.bin").unwrap_or_else(|| {
            let mut path = heapless::String::<64>::new();
            let _ = path.push_str("/defmt.bin");
            path
        });
        let mut staging = DefmtStaging::new();

        loop {
            {
                let mounted = MountedStorage { fs: &fs };
                staging.drain(&mounted, defmt_path.as_str(), &mut consumer);
            }

            match select(FS.run(&mut fs), consumer.wait_for_log()).await {
                Either::First(_) => unreachable!(),
                Either::Second(grant) => {
                    let data = grant.buf();
                    let len = data.len();
                    let mounted = MountedStorage { fs: &fs };
                    staging.stage(&mounted, defmt_path.as_str(), data);
                    grant.release(len);
                }
            }
        }
    }
}

#[cfg(not(feature = "storage"))]
mod backend {
    use super::*;

    pub async fn save<const N: usize>(
        _key: &'static str,
        _data: [u8; N],
        _len: usize,
    ) -> SaveStatus {
        UnavailableStorage.save("", &[])
    }

    #[embassy_executor::task]
    pub async fn task(_flash: &'static mut BoardFlash, mut consumer: DefmtConsumer) -> ! {
        crate::params::load_all(&super::UnavailableStorage);
        CONFIG_READY.signal(());
        defmt::info!("storage: disabled, using in-memory defaults");

        loop {
            let grant = consumer.wait_for_log().await;
            let len = grant.buf().len();
            grant.release(len);
        }
    }
}
