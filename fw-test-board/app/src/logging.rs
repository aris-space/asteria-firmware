use core::future::pending;
use defmt_brtt::DefmtConsumer;
use embedded_utils::fmt::*;
use littlefs2::driver::Storage;
use littlefs2::fs::{Allocation, Filesystem};
use littlefs2::path::{Path, PathBuf};

use crate::board::FlashAdapter;

const LOG_DIR_PREFIX: &[u8; 5] = b"/log_";
const LOG_DIR_PATH_MAX_LEN: usize = 16;

#[embassy_executor::task]
pub async fn logging_task() -> ! {
    info!("logging task: startup");
    let (storage, consumer): (FlashAdapter<'static>, DefmtConsumer) = {
        let mut board = crate::board::BOARD
            .try_get()
            .expect("BOARD not initialized")
            .try_lock()
            .expect("BOARD mutex busy while taking logging resources");

        let storage = board.flash.take().expect("flash not available");
        let consumer = board
            .defmt_log
            .take()
            .expect("defmt consumer not available");
        (storage, consumer)
    };
    info!("logging task: acquired all resources");

    setup_and_loop(storage, consumer).await;
    error!("logging task: persistent storage unavailable, dropping log frames without saving");

    pending::<()>().await;
    embedded_utils::unreachable!();
}

async fn setup_and_loop(mut storage: FlashAdapter<'static>, mut consumer: DefmtConsumer) {
    debug!("logging task: mounting littlefs");
    let mut alloc = Allocation::new();
    let fs = match Filesystem::mount(&mut alloc, &mut storage) {
        Ok(fs) => fs,
        Err(_) => {
            warn!("logging task: mount failed, formatting");
            if Filesystem::format(&mut storage).is_err() {
                warn!("logging task: format failed");
                return;
            }

            match Filesystem::mount(&mut alloc, &mut storage) {
                Ok(fs) => fs,
                Err(_) => {
                    warn!("logging task: mount failed after format");
                    return;
                }
            }
        }
    };
    info!("logging task: littlefs mounted successfully");
    let defmt_path = match prepare_log_session(&fs) {
        Some(path) => path,
        None => {
            warn!("logging task: session setup failed, dropping frames");
            info!("logging task: running without flash persistence");
            return;
        }
    };

    info!("logging task: flash logging active");

    loop {
        let grant = consumer.wait_for_log().await;
        let data = grant.buf();
        let len = data.len();

        if len > 0 && append_to_file(&fs, defmt_path.as_path(), data).is_err() {
            // warn!("logging task: append failed");
            // todo figure out a way to log.
        }

        grant.release(len);
    }
}

fn prepare_log_session<S: Storage>(fs: &Filesystem<S>) -> Option<PathBuf> {
    let log_index = select_next_log_index(fs).ok()?;
    let mut index_buf = itoa::Buffer::new();
    let log_dir = log_dir_path(log_index, &mut index_buf)?;

    info!("logging task: logging to session dir /log_{}", log_index);

    if fs.exists(log_dir.as_path()) {
        debug!("logging task: clearing existing session directory");
        if fs.remove_dir_all(log_dir.as_path()).is_err() {
            warn!("logging task: failed to clear session directory");
            return None;
        }
    }

    if fs.create_dir(log_dir.as_path()).is_err() {
        warn!("logging task: failed to create session directory");
        return None;
    }

    let defmt_file = Path::from_bytes_with_nul(b"defmt.bin\0").ok()?;
    let mut defmt_path = PathBuf::from(log_dir.as_path());
    defmt_path.push(defmt_file);
    if fs.write(defmt_path.as_path(), &[]).is_err() {
        warn!("logging task: failed to create defmt.bin");
        return None;
    }

    let build_info_file = Path::from_bytes_with_nul(b"build_info.txt\0").ok()?;
    let mut build_info_path = PathBuf::from(log_dir.as_path());
    build_info_path.push(build_info_file);
    if write_build_info(fs, build_info_path.as_path()).is_err() {
        warn!("logging task: failed to write build_info.txt");
        return None;
    }
    Some(defmt_path)
}

fn append_to_file<S: Storage>(
    fs: &Filesystem<S>,
    path: &Path,
    data: &[u8],
) -> littlefs2::io::Result<()> {
    fs.open_file_with_options_and_then(
        |options| options.write(true).create(true).append(true),
        path,
        |file| {
            file.write(data)?;
            Ok(())
        },
    )
}

fn write_build_info<S: Storage>(fs: &Filesystem<S>, path: &Path) -> littlefs2::io::Result<()> {
    fs.open_file_with_options_and_then(
        |options| options.write(true).create(true).truncate(true),
        path,
        |file| {
            file.write(b"pkg_name=")?;
            file.write(crate::built_info::PKG_NAME.as_bytes())?;
            file.write(b"\nversion=")?;
            file.write(crate::built_info::PKG_VERSION.as_bytes())?;
            file.write(b"\nprofile=")?;
            file.write(crate::built_info::PROFILE.as_bytes())?;
            file.write(b"\ntarget=")?;
            file.write(crate::built_info::TARGET.as_bytes())?;
            file.write(b"\ngit_commit_short=")?;
            match crate::built_info::GIT_COMMIT_HASH_SHORT {
                Some(hash) => {
                    file.write(hash.as_bytes())?;
                }
                None => {
                    file.write(b"none")?;
                }
            }
            file.write(b"\ngit_dirty=")?;
            match crate::built_info::GIT_DIRTY {
                Some(true) => {
                    file.write(b"true")?;
                }
                Some(false) => {
                    file.write(b"false")?;
                }
                None => {
                    file.write(b"none")?;
                }
            }
            file.write(b"\nfeatures=")?;
            file.write(crate::built_info::FEATURES_LOWERCASE_STR.as_bytes())?;
            file.write(b"\nartifact_timestamp_ms=")?;
            match crate::built_info::ASTERIA_ARTIFACT_TIMESTAMP_MS {
                Some(ts) => file.write(ts.as_bytes())?,
                None => file.write(b"none")?,
            };
            file.write(b"\nrustc=")?;
            file.write(crate::built_info::RUSTC.as_bytes())?;
            file.write(b"\nrustc_version=")?;
            file.write(crate::built_info::RUSTC_VERSION.as_bytes())?;
            file.write(b"\n")?;
            Ok(())
        },
    )
}

fn select_next_log_index<S: Storage>(fs: &Filesystem<S>) -> Result<u32, ()> {
    let root = Path::from_bytes_with_nul(b"/\0").map_err(|_| ())?;
    let mut max_index: Option<u32> = None;

    if fs
        .read_dir_and_then(root, |entries| {
            for entry in entries {
                let entry = entry?;
                if !entry.file_type().is_dir() {
                    continue;
                }

                if let Some(index) = parse_log_dir_index(entry.file_name().as_ref()) {
                    max_index = Some(match max_index {
                        Some(current_max) => current_max.max(index),
                        None => index,
                    });
                }
            }
            Ok(())
        })
        .is_err()
    {
        warn!("logging task: failed to scan root directory");
        return Err(());
    }

    Ok(max_index.map_or(0, |n| n.saturating_add(1)))
}

fn parse_log_dir_index(name: &str) -> Option<u32> {
    let digits = name.strip_prefix("log_")?;
    if digits.is_empty() {
        return None;
    }

    let mut value: u32 = 0;
    for ch in digits.as_bytes() {
        if !ch.is_ascii_digit() {
            return None;
        }
        value = value.checked_mul(10)?;
        value = value.checked_add((ch - b'0') as u32)?;
    }

    Some(value)
}

fn log_dir_path(index: u32, buf: &mut itoa::Buffer) -> Option<PathBuf> {
    let digits = buf.format(index).as_bytes();
    let len = LOG_DIR_PREFIX.len() + digits.len();
    if len > LOG_DIR_PATH_MAX_LEN {
        return None;
    }

    let mut path_bytes = [0u8; LOG_DIR_PATH_MAX_LEN];
    path_bytes[..LOG_DIR_PREFIX.len()].copy_from_slice(LOG_DIR_PREFIX);
    path_bytes[LOG_DIR_PREFIX.len()..len].copy_from_slice(digits);

    PathBuf::try_from(&path_bytes[..len]).ok()
}
