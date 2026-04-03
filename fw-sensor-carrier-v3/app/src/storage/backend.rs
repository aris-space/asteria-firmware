use defmt_brtt::DefmtConsumer;

use crate::resources::flash::BoardFlash;

async fn discard_logs_forever(mut consumer: DefmtConsumer) -> ! {
    loop {
        let grant = consumer.wait_for_log().await;
        let len = grant.buf().len();
        grant.release(len);
    }
}

#[cfg(feature = "storage")]
mod imp {
    use embassy_futures::select::{Either, select};
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::rpc_service::RpcService;
    use generic_array::typenum::{U1, U256};
    use littlefs2::fs::{Allocation, Filesystem};
    use littlefs2::io::Error;
    use littlefs2::path;
    use littlefs2::path::PathBuf;
    use static_cell::StaticCell;
    use w25q256jv::LittlefsAdapter;

    use crate::resources::flash::{FixedHighPin, FlashDevice};
    use crate::storage::session::{
        DefmtStaging, active_session_file_path, default_log_path, prepare_session,
    };

    use super::super::{
        CONFIG_READY, FileStorage, FileWriteMode, KeyRead, KeyStorage, SaveStatus, StorageResult,
        UnavailableStorage, is_available, set_available, set_session_index,
    };
    use super::{BoardFlash, DefmtConsumer, discard_logs_forever};

    type Adapter = LittlefsAdapter<'static, FlashDevice, FixedHighPin, FixedHighPin, U256, U1>;
    type Fs = Filesystem<'static, Adapter>;

    static ADAPTER: StaticCell<Adapter> = StaticCell::new();
    static ALLOC: StaticCell<Allocation<Adapter>> = StaticCell::new();
    static FS: RpcService<CriticalSectionRawMutex, Fs, 512> = RpcService::new();

    struct MountedStorage<'a> {
        fs: &'a Fs,
    }

    impl MountedStorage<'_> {
        fn config_path(key: &str) -> Option<PathBuf> {
            let mut path = heapless::String::<64>::new();
            use core::fmt::Write as _;
            write!(path, "/params/{key}.bin").ok()?;
            PathBuf::try_from(path.as_bytes()).ok()
        }

        fn path(path: &str) -> Option<PathBuf> {
            PathBuf::try_from(path.as_bytes()).ok()
        }
    }

    impl KeyStorage for MountedStorage<'_> {
        fn read_key(&self, key: &str, out: &mut [u8]) -> KeyRead {
            let Some(path) = Self::config_path(key) else {
                return KeyRead::Unavailable;
            };

            match self.fs.open_file_and_then(&path, |file| file.read(out)) {
                Ok(len) => KeyRead::Found(len),
                Err(Error::NO_SUCH_ENTRY) => KeyRead::Missing,
                Err(_) => KeyRead::Unavailable,
            }
        }

        fn write_key(&self, key: &str, data: &[u8]) -> StorageResult {
            let Some(path) = Self::config_path(key) else {
                return Err(super::super::StorageUnavailable);
            };

            let _ = self.fs.create_dir_all(path!("/params"));
            match self.fs.open_file_with_options_and_then(
                |options| options.create(true).truncate(true),
                &path,
                |file| file.write(data),
            ) {
                Ok(_) => Ok(()),
                Err(_) => Err(super::super::StorageUnavailable),
            }
        }
    }

    impl FileStorage for MountedStorage<'_> {
        fn visit_dir(&self, path: &str, visitor: &mut dyn FnMut(&str)) -> StorageResult {
            let Some(path) = Self::path(path) else {
                return Err(super::super::StorageUnavailable);
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
                .map(|_| ())
                .map_err(|_| super::super::StorageUnavailable)
        }

        fn ensure_dir(&self, path: &str) -> StorageResult {
            let Some(path) = Self::path(path) else {
                return Err(super::super::StorageUnavailable);
            };

            self.fs
                .create_dir(&path)
                .map(|_| ())
                .map_err(|_| super::super::StorageUnavailable)
        }

        fn write_file(&self, path: &str, data: &[u8], mode: FileWriteMode) -> StorageResult {
            let Some(path) = Self::path(path) else {
                return Err(super::super::StorageUnavailable);
            };

            self.fs
                .open_file_with_options_and_then(
                    |options| {
                        options.create(true);
                        match mode {
                            FileWriteMode::Overwrite => options.truncate(true),
                            FileWriteMode::Append => options.append(true),
                        }
                    },
                    &path,
                    |file| file.write(data),
                )
                .map(|_| ())
                .map_err(|_| super::super::StorageUnavailable)
        }
    }

    pub(crate) async fn persist_key<const N: usize>(
        key: &'static str,
        data: [u8; N],
        len: usize,
    ) -> SaveStatus {
        if !is_available() || len > N {
            return SaveStatus::RuntimeOnly;
        }

        FS.call(move |fs| MountedStorage { fs }.write_key(key, &data[..len]))
            .await
            .map(|_| SaveStatus::Persisted)
            .unwrap_or(SaveStatus::RuntimeOnly)
    }

    #[embassy_executor::task]
    pub(crate) async fn task(flash: &'static mut BoardFlash, consumer: DefmtConsumer) -> ! {
        let adapter = ADAPTER.init(LittlefsAdapter::<_, _, _, U256, U1>::new(flash));
        let alloc = ALLOC.init(Filesystem::allocate());

        if Filesystem::mount(alloc, adapter).is_err() {
            defmt::info!("storage: mount failed, formatting");
            let _ = Filesystem::format(adapter);
        }

        let Ok(mut fs) = Filesystem::mount(alloc, adapter) else {
            defmt::warn!("storage: unavailable, continuing with in-memory defaults");
            crate::params::load_all(&UnavailableStorage);
            CONFIG_READY.signal(());
            discard_logs_forever(consumer).await
        };

        set_available(true);
        defmt::info!("storage: backend mounted");

        {
            let mounted = MountedStorage { fs: &fs };
            crate::params::load_all(&mounted);

            if let Some(index) = prepare_session(&mounted) {
                set_session_index(index);
            }
        }

        CONFIG_READY.signal(());
        defmt::info!("storage: config initialized");

        let defmt_path = active_session_file_path("defmt.bin").unwrap_or_else(default_log_path);
        let mut consumer = consumer;
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
mod imp {
    use super::{BoardFlash, DefmtConsumer, discard_logs_forever};
    use crate::storage::{CONFIG_READY, SaveStatus, UnavailableStorage};

    pub(crate) async fn persist_key<const N: usize>(
        _key: &'static str,
        _data: [u8; N],
        _len: usize,
    ) -> SaveStatus {
        SaveStatus::RuntimeOnly
    }

    #[embassy_executor::task]
    pub(crate) async fn task(_flash: &'static mut BoardFlash, consumer: DefmtConsumer) -> ! {
        crate::params::load_all(&UnavailableStorage);
        CONFIG_READY.signal(());
        defmt::info!("storage: disabled, using in-memory defaults");
        discard_logs_forever(consumer).await
    }
}

pub(crate) use imp::{persist_key, task};
