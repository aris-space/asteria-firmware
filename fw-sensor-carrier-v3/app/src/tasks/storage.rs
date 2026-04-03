#[cfg(feature = "storage")]
use core::fmt::Write as _;
#[cfg(feature = "storage")]
use core::sync::atomic::AtomicU32;
use core::sync::atomic::{AtomicBool, Ordering};

use defmt_brtt::DefmtConsumer;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::params::{BackendLoad, ConfigBackend, SaveStatus};
use crate::resources::flash::BoardFlash;

pub static CONFIG_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[allow(dead_code)]
static AVAILABLE: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "storage")]
static SESSION_INDEX: AtomicU32 = AtomicU32::new(u32::MAX);

#[allow(dead_code)]
pub fn is_available() -> bool {
    AVAILABLE.load(Ordering::Relaxed)
}

#[allow(dead_code)]
pub async fn save(key: &'static str, data: &[u8]) -> SaveStatus {
    backend::save(key, data).await
}

#[cfg(feature = "storage")]
pub fn workdir_path(filename: &str) -> Option<littlefs2::path::PathBuf> {
    let mut path = heapless::String::<64>::new();
    write!(path, "/log_{}/{}", session_idx()?, filename).ok()?;
    littlefs2::path::PathBuf::try_from(path.as_bytes()).ok()
}

#[cfg(feature = "storage")]
fn session_idx() -> Option<u32> {
    match SESSION_INDEX.load(Ordering::Relaxed) {
        u32::MAX => None,
        index => Some(index),
    }
}

pub use backend::task;

struct UnavailableBackend;

impl ConfigBackend for UnavailableBackend {
    fn load(&self, _key: &str, _out: &mut [u8]) -> BackendLoad {
        BackendLoad::Unavailable
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

    struct MountedBackend<'a> {
        fs: &'a Fs,
    }

    impl ConfigBackend for MountedBackend<'_> {
        fn load(&self, key: &str, out: &mut [u8]) -> BackendLoad {
            let Some(path) = config_path(key) else {
                return BackendLoad::Unavailable;
            };

            match self.fs.open_file_and_then(&path, |file| file.read(out)) {
                Ok(len) => BackendLoad::Loaded(len),
                Err(Error::NO_SUCH_ENTRY) => BackendLoad::Missing,
                Err(_) => BackendLoad::Unavailable,
            }
        }
    }

    fn config_path(key: &str) -> Option<PathBuf> {
        let mut path = heapless::String::<64>::new();
        write!(path, "/params/{}.bin", key).ok()?;
        PathBuf::try_from(path.as_bytes()).ok()
    }

    fn prepare_session(fs: &Fs) -> Option<u32> {
        let mut max_index: Option<u32> = None;

        let _ = fs.read_dir_and_then(path!("/"), |dir| {
            for entry in dir {
                let entry = entry?;
                let name = entry.file_name().as_str_ref_with_trailing_nul();
                let name = name.trim_end_matches('\0');
                if let Some(suffix) = name.strip_prefix("log_")
                    && let Ok(n) = suffix.parse::<u32>()
                {
                    max_index = Some(max_index.map_or(n, |m| m.max(n)));
                }
            }
            Ok(())
        });

        let next = max_index.map_or(0, |n| n + 1);
        let mut dir_name = heapless::String::<32>::new();
        write!(dir_name, "/log_{}", next).ok()?;

        fs.create_dir(&PathBuf::try_from(dir_name.as_bytes()).ok()?)
            .ok()?;

        let mut info_path = heapless::String::<64>::new();
        write!(info_path, "{}/build_info.txt", dir_name.as_str()).ok()?;

        let _ = fs.create_file_and_then(&PathBuf::try_from(info_path.as_bytes()).ok()?, |file| {
            use crate::built;
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
            file.write(buf.as_bytes())?;
            Ok(())
        });

        defmt::info!("storage: session {}", dir_name.as_str());
        Some(next)
    }

    const STAGING_SIZE: usize = 512;

    struct DefmtStaging {
        buf: [u8; STAGING_SIZE],
        len: usize,
    }

    impl DefmtStaging {
        fn new() -> Self {
            Self {
                buf: [0u8; STAGING_SIZE],
                len: 0,
            }
        }

        fn stage(&mut self, fs: &Fs, path: &PathBuf, data: &[u8]) {
            if self.len + data.len() > STAGING_SIZE {
                self.flush(fs, path);
            }
            let count = data.len().min(STAGING_SIZE - self.len);
            self.buf[self.len..self.len + count].copy_from_slice(&data[..count]);
            self.len += count;
        }

        fn flush(&mut self, fs: &Fs, path: &PathBuf) {
            if self.len == 0 {
                return;
            }

            let _ = fs.open_file_with_options_and_then(
                |options| options.append(true).create(true),
                path,
                |file| file.write(&self.buf[..self.len]),
            );
            self.len = 0;
        }

        fn drain(&mut self, fs: &Fs, path: &PathBuf, consumer: &mut DefmtConsumer) {
            while let Ok(grant) = consumer.read() {
                let data = grant.buf();
                let len = data.len();
                self.stage(fs, path, data);
                grant.release(len);
            }
            self.flush(fs, path);
        }
    }

    pub async fn save(key: &'static str, data: &[u8]) -> SaveStatus {
        if !super::is_available() {
            return SaveStatus::RuntimeOnly;
        }

        let mut buf = [0u8; 512];
        if data.len() > buf.len() {
            return SaveStatus::RuntimeOnly;
        }
        let count = data.len();
        buf[..count].copy_from_slice(data);

        FS.call(move |fs| {
            let Some(path) = config_path(key) else {
                return SaveStatus::RuntimeOnly;
            };

            let _ = fs.create_dir_all(littlefs2::path!("/params"));
            match fs.open_file_with_options_and_then(
                |options| options.create(true).truncate(true),
                &path,
                |file| file.write(&buf[..count]),
            ) {
                Ok(_) => SaveStatus::Persisted,
                Err(_) => SaveStatus::RuntimeOnly,
            }
        })
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
            crate::params::load_all(&super::UnavailableBackend);
            CONFIG_READY.signal(());

            loop {
                let grant = consumer.wait_for_log().await;
                let len = grant.buf().len();
                grant.release(len);
            }
        };

        AVAILABLE.store(true, Ordering::Relaxed);
        defmt::info!("storage: filesystem mounted");

        crate::params::load_all(&MountedBackend { fs: &fs });

        if let Some(index) = prepare_session(&fs) {
            SESSION_INDEX.store(index, Ordering::Relaxed);
        }

        CONFIG_READY.signal(());
        defmt::info!("storage: config initialized");

        let defmt_path = super::workdir_path("defmt.bin")
            .unwrap_or_else(|| PathBuf::try_from(b"/defmt.bin".as_slice()).unwrap());
        let mut staging = DefmtStaging::new();

        loop {
            staging.drain(&fs, &defmt_path, &mut consumer);

            match select(FS.run(&mut fs), consumer.wait_for_log()).await {
                Either::First(_) => unreachable!(),
                Either::Second(grant) => {
                    let data = grant.buf();
                    let len = data.len();
                    staging.stage(&fs, &defmt_path, data);
                    grant.release(len);
                }
            }
        }
    }
}

#[cfg(not(feature = "storage"))]
mod backend {
    use super::*;

    #[allow(dead_code)]
    pub async fn save(_key: &'static str, _data: &[u8]) -> SaveStatus {
        SaveStatus::RuntimeOnly
    }

    #[embassy_executor::task]
    pub async fn task(_flash: &'static mut BoardFlash, mut consumer: DefmtConsumer) -> ! {
        crate::params::load_all(&super::UnavailableBackend);
        CONFIG_READY.signal(());
        defmt::info!("storage: disabled, using in-memory defaults");

        loop {
            let grant = consumer.wait_for_log().await;
            let len = grant.buf().len();
            grant.release(len);
        }
    }
}
