use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt_brtt::DefmtConsumer;
use embassy_sync::blocking_mutex::ThreadModeMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use heapless::{String as HString, Vec as HVec};
use littlefs2::driver::Storage;
use littlefs2::fs::{Allocation, FileType, Filesystem};
use littlefs2::io::SeekFrom;
use littlefs2::path::{Path, PathBuf};
use static_cell::StaticCell;
use wire_types::*;

use crate::board::FlashAdapter;

// Large files are transferred across many fs/read requests; this value is per-response chunk size.
// Keep it conservative for raw-usb transport robustness.
const FS_MAX_CHUNK: usize = 512;
const LOG_DIR_PREFIX: &[u8; 5] = b"/log_";
const LOG_DIR_PATH_MAX_LEN: usize = 16;

static LOG_STORAGE: StaticCell<FlashAdapter<'static>> = StaticCell::new();
static LOG_ALLOC: StaticCell<Allocation<FlashAdapter<'static>>> = StaticCell::new();

struct FsState {
    fs: Filesystem<'static, FlashAdapter<'static>>,
    defmt_path: PathBuf,
}

static FS_STATE: ThreadModeMutex<RefCell<Option<FsState>>> =
    ThreadModeMutex::new(RefCell::new(None));

struct FsMailbox {
    req: Channel<CriticalSectionRawMutex, FsRequest, 1>,
    resp: Signal<CriticalSectionRawMutex, FsResponse>,
    call_lock: Mutex<CriticalSectionRawMutex, ()>,
}

impl FsMailbox {
    const fn new() -> Self {
        Self {
            req: Channel::new(),
            resp: Signal::new(),
            call_lock: Mutex::new(()),
        }
    }

    async fn request(&self, req: FsRequest) -> FsResponse {
        let _guard = self.call_lock.lock().await;
        self.resp.reset();
        self.req.send(req).await;
        self.resp.wait().await
    }

    async fn wait_request(&self) -> FsRequest {
        self.req.receive().await
    }

    fn respond(&self, resp: FsResponse) {
        self.resp.signal(resp);
    }
}

static FS_MAILBOX: FsMailbox = FsMailbox::new();
static FS_EPOCH: AtomicU32 = AtomicU32::new(1);

pub enum FsRequest {
    Info(FsInfoReq),
    ListDir(FsListDirReq),
    Stat(FsStatReq),
    ReadFile(FsReadFileReq),
    Remove(FsRemoveReq),
    EraseStorage(FsEraseStorageReq),
}

pub enum FsResponse {
    Info(FsInfoResp),
    ListDir(FsListDirResp),
    Stat(FsStatResp),
    ReadFile(FsReadFileResp),
    Remove(FsRemoveResp),
    EraseStorage(FsEraseStorageResp),
}

pub fn fs_epoch() -> u32 {
    FS_EPOCH.load(Ordering::Relaxed)
}

pub async fn fs_request(req: FsRequest) -> FsResponse {
    FS_MAILBOX.request(req).await
}

#[embassy_executor::task]
pub async fn logging_task() -> ! {
    embedded_utils::info!("logging task: startup");
    let mut consumer: DefmtConsumer = {
        let mut board = crate::board::BOARD
            .try_get()
            .expect("BOARD not initialized")
            .try_lock()
            .expect("BOARD mutex busy while taking defmt consumer");

        board
            .defmt_log
            .take()
            .expect("defmt consumer not available")
    };
    embedded_utils::info!("logging task: acquired all resources");

    let mut dropped_writes: u32 = 0;

    loop {
        let grant = consumer.wait_for_log().await;
        let data = grant.buf();
        let len = data.len();

        if len > 0 {
            match with_fs_mut(|state| append_to_file(&state.fs, state.defmt_path.as_path(), data)) {
                Some(Ok(())) => {}
                Some(Err(_)) => {
                    embedded_utils::fmt::warn!("logging task: append failed");
                }
                None => {
                    dropped_writes = dropped_writes.wrapping_add(1);
                    if dropped_writes % 128 == 0 {
                        embedded_utils::fmt::warn!(
                            "logging task: dropped {} writes (fs unavailable)",
                            dropped_writes
                        );
                    }
                }
            }
        }

        grant.release(len);
    }
}

#[embassy_executor::task]
pub async fn fs_worker() -> ! {
    embedded_utils::info!("fs worker: startup");
    let flash: FlashAdapter<'static> = {
        let mut board = crate::board::BOARD
            .try_get()
            .expect("BOARD not initialized")
            .try_lock()
            .expect("BOARD mutex busy while taking flash");

        board.flash.take().expect("flash not available")
    };
    embedded_utils::info!("fs worker: acquired all resources");

    let state = init_filesystem(flash);
    FS_STATE.lock(|cell| {
        *cell.borrow_mut() = state;
    });

    if fs_available() {
        embedded_utils::info!("fs worker: ready");
    } else {
        embedded_utils::fmt::warn!("fs worker: running degraded (no persistent storage)");
    }

    loop {
        let req = FS_MAILBOX.wait_request().await;
        let resp = match req {
            FsRequest::EraseStorage(_req) => handle_erase_storage(),
            req => match with_fs_req(req, |state, req| handle_fs_request(&mut state.fs, req)) {
                Ok(resp) => resp,
                Err(req) => degraded_fs_response(req),
            },
        };
        FS_MAILBOX.respond(resp);
    }
}

fn fs_available() -> bool {
    FS_STATE.lock(|cell| cell.borrow().is_some())
}

fn with_fs_mut<R>(f: impl FnOnce(&mut FsState) -> R) -> Option<R> {
    FS_STATE.lock(|cell| {
        let mut borrow = cell.borrow_mut();
        let state = borrow.as_mut()?;
        Some(f(state))
    })
}

fn with_fs_req<T, R>(input: T, f: impl FnOnce(&mut FsState, T) -> R) -> Result<R, T> {
    FS_STATE.lock(|cell| {
        let mut borrow = cell.borrow_mut();
        let Some(state) = borrow.as_mut() else {
            return Err(input);
        };
        Ok(f(state, input))
    })
}

fn init_filesystem(flash: FlashAdapter<'static>) -> Option<FsState> {
    embedded_utils::debug!("fs worker: mounting littlefs");

    let storage = LOG_STORAGE.init(flash);

    if !Filesystem::is_mountable(storage) {
        embedded_utils::fmt::warn!("fs worker: mount check failed, formatting");
        if Filesystem::format(storage).is_err() {
            embedded_utils::fmt::warn!("fs worker: format failed");
            return None;
        }
    }

    let alloc = LOG_ALLOC.init(Allocation::new());
    let fs = match Filesystem::mount(alloc, storage) {
        Ok(fs) => fs,
        Err(_) => {
            embedded_utils::fmt::warn!("fs worker: mount failed after format");
            return None;
        }
    };

    embedded_utils::info!("fs worker: littlefs mounted successfully");

    let defmt_path = match prepare_log_session(&fs) {
        Some(path) => path,
        None => {
            embedded_utils::fmt::warn!("fs worker: session setup failed");
            return None;
        }
    };

    Some(FsState { fs, defmt_path })
}

fn degraded_fs_response(req: FsRequest) -> FsResponse {
    match req {
        FsRequest::Info(_req) => FsResponse::Info(fs_info_err(FsError::Io)),
        FsRequest::ListDir(_req) => FsResponse::ListDir(fs_list_dir_err(FsError::Io)),
        FsRequest::Stat(_req) => FsResponse::Stat(fs_stat_err(FsError::Io)),
        FsRequest::ReadFile(req) => FsResponse::ReadFile(fs_read_file_err(FsError::Io, req.offset)),
        FsRequest::Remove(_req) => FsResponse::Remove(fs_remove_err(FsError::Io)),
        FsRequest::EraseStorage(_req) => {
            FsResponse::EraseStorage(fs_erase_storage_err(FsError::Io))
        }
    }
}

fn handle_fs_request<S: Storage>(fs: &mut Filesystem<'_, S>, req: FsRequest) -> FsResponse {
    match req {
        FsRequest::Info(_req) => FsResponse::Info(handle_fs_info()),
        FsRequest::ListDir(req) => FsResponse::ListDir(handle_list_dir(
            fs,
            req.path,
            req.cursor,
            req.max_entries,
            req.expected_epoch,
        )),
        FsRequest::Stat(req) => FsResponse::Stat(handle_stat(fs, req.path, req.expected_epoch)),
        FsRequest::ReadFile(req) => FsResponse::ReadFile(handle_read_file(
            fs,
            req.path,
            req.offset,
            req.len,
            req.expected_epoch,
        )),
        FsRequest::Remove(req) => {
            FsResponse::Remove(handle_remove(fs, req.path, req.expected_epoch))
        }
        FsRequest::EraseStorage(_req) => {
            FsResponse::EraseStorage(fs_erase_storage_err(FsError::Io))
        }
    }
}

fn handle_erase_storage() -> FsResponse {
    let resp = FS_STATE.lock(|cell| {
        let mut slot = cell.borrow_mut();
        let Some(state) = slot.take() else {
            return fs_erase_storage_err(FsError::Io);
        };

        let (alloc, storage) = state.fs.into_inner();
        if Filesystem::format(storage).is_err() {
            embedded_utils::fmt::warn!("fs worker: erase format failed");
            return fs_erase_storage_err(FsError::Io);
        }

        let fs = match Filesystem::mount(alloc, storage) {
            Ok(fs) => fs,
            Err(_) => {
                embedded_utils::fmt::warn!("fs worker: remount failed after erase");
                return fs_erase_storage_err(FsError::Io);
            }
        };

        let defmt_path = match prepare_log_session(&fs) {
            Some(path) => path,
            None => {
                embedded_utils::fmt::warn!("fs worker: session setup failed after erase");
                return fs_erase_storage_err(FsError::Io);
            }
        };

        *slot = Some(FsState { fs, defmt_path });

        FsEraseStorageResp {
            err: FsError::Ok,
            epoch: fs_epoch(),
        }
    });

    FsResponse::EraseStorage(resp)
}

fn note_fs_mutation() {
    let _ = FS_EPOCH.fetch_add(1, Ordering::Relaxed);
}

fn epoch_matches(expected: Option<FsEpoch>) -> bool {
    match expected {
        Some(e) => e == fs_epoch(),
        None => true,
    }
}

fn fs_info_ok() -> FsInfoResp {
    FsInfoResp {
        err: FsError::Ok,
        epoch: fs_epoch(),
        max_chunk: FS_MAX_CHUNK as u16,
        max_dir_entries: FS_DIR_PAGE_CAP as u16,
    }
}

fn fs_info_err(err: FsError) -> FsInfoResp {
    FsInfoResp {
        err,
        epoch: fs_epoch(),
        max_chunk: FS_MAX_CHUNK as u16,
        max_dir_entries: FS_DIR_PAGE_CAP as u16,
    }
}

fn fs_list_dir_err(err: FsError) -> FsListDirResp {
    FsListDirResp {
        err,
        epoch: fs_epoch(),
        entries: HVec::new(),
        next_cursor: 0,
    }
}

fn fs_stat_err(err: FsError) -> FsStatResp {
    FsStatResp {
        err,
        epoch: fs_epoch(),
        kind: FsNodeKind::File,
        size_bytes: 0,
    }
}

fn fs_read_file_err(err: FsError, offset: u32) -> FsReadFileResp {
    FsReadFileResp {
        err,
        epoch: fs_epoch(),
        offset,
        data: HVec::new(),
        done: false,
    }
}

fn fs_remove_err(err: FsError) -> FsRemoveResp {
    FsRemoveResp {
        err,
        epoch: fs_epoch(),
    }
}

fn fs_erase_storage_err(err: FsError) -> FsEraseStorageResp {
    FsEraseStorageResp {
        err,
        epoch: fs_epoch(),
    }
}

fn normalize_req_path(input: &str) -> Result<HString<FS_PATH_CAP>, FsError> {
    let mut out = HString::<FS_PATH_CAP>::new();
    out.push('/').map_err(|_| FsError::Io)?;

    let mut first = true;
    for seg in input.trim().split('/').filter(|s| !s.is_empty()) {
        if seg == "." || seg == ".." || seg.contains('\\') {
            return Err(FsError::NotFound);
        }

        if !first {
            out.push('/').map_err(|_| FsError::NotFound)?;
        }
        out.push_str(seg).map_err(|_| FsError::NotFound)?;
        first = false;
    }

    Ok(out)
}

fn to_lfs_path(path: &HString<FS_PATH_CAP>) -> Result<littlefs2::path::PathBuf, FsError> {
    littlefs2::path::PathBuf::try_from(path.as_str()).map_err(|_| FsError::NotFound)
}

fn path_exists_as_file<S: littlefs2::driver::Storage>(
    fs: &Filesystem<'_, S>,
    path: &littlefs2::path::Path,
) -> bool {
    fs.open_file_with_options_and_then(
        |opts| opts.read(true),
        path,
        |_file| -> littlefs2::io::Result<()> { Ok(()) },
    )
    .is_ok()
}

fn path_exists_as_dir<S: littlefs2::driver::Storage>(
    fs: &Filesystem<'_, S>,
    path: &littlefs2::path::Path,
) -> bool {
    fs.read_dir_and_then(path, |_dir| -> littlefs2::io::Result<()> { Ok(()) })
        .is_ok()
}

fn file_size_bytes<S: littlefs2::driver::Storage>(
    fs: &Filesystem<'_, S>,
    path: &littlefs2::path::Path,
) -> Option<u32> {
    let mut size = None;
    let _ = fs.open_file_with_options_and_then(
        |opts| opts.read(true),
        path,
        |file| -> littlefs2::io::Result<()> {
            size = Some(file.seek(SeekFrom::End(0))?);
            Ok(())
        },
    );
    size.map(|n| n.min(u32::MAX as usize) as u32)
}

fn handle_fs_info() -> FsInfoResp {
    fs_info_ok()
}

fn handle_list_dir<S: littlefs2::driver::Storage>(
    fs: &Filesystem<'_, S>,
    req_path: HString<FS_PATH_CAP>,
    cursor: u32,
    max_entries: u16,
    expected_epoch: Option<FsEpoch>,
) -> FsListDirResp {
    if !epoch_matches(expected_epoch) {
        return fs_list_dir_err(FsError::EpochMismatch);
    }

    let norm_path = match normalize_req_path(req_path.as_str()) {
        Ok(p) => p,
        Err(err) => return fs_list_dir_err(err),
    };

    let lfs_path = match to_lfs_path(&norm_path) {
        Ok(p) => p,
        Err(err) => return fs_list_dir_err(err),
    };

    let page_cap = core::cmp::min(max_entries.max(1) as usize, FS_DIR_PAGE_CAP);
    let mut entries: HVec<FsDirEntry, FS_DIR_PAGE_CAP> = HVec::new();
    let mut total_entries = 0usize;
    let mut page_scanned = 0usize;

    let list_res = fs.read_dir_and_then(lfs_path.as_path(), |dir| -> littlefs2::io::Result<()> {
        for entry in dir {
            let entry = entry?;
            let name_str = entry.file_name().as_str();
            if name_str == "." || name_str == ".." {
                continue;
            }

            if total_entries >= cursor as usize && page_scanned < page_cap {
                page_scanned = page_scanned.saturating_add(1);

                let mut name = HString::<FS_NAME_CAP>::new();
                if name.push_str(name_str).is_ok() {
                    let kind = if entry.file_type() == FileType::Dir {
                        FsNodeKind::Dir
                    } else {
                        FsNodeKind::File
                    };
                    let _ = entries.push(FsDirEntry { kind, name });
                }
            }

            total_entries = total_entries.saturating_add(1);
        }
        Ok(())
    });

    if list_res.is_err() {
        if path_exists_as_file(fs, lfs_path.as_path()) {
            return fs_list_dir_err(FsError::NotDir);
        }
        return fs_list_dir_err(FsError::NotFound);
    }

    let consumed = page_scanned as u32;
    let next_cursor = if total_entries as u32 > cursor.saturating_add(consumed) {
        cursor.saturating_add(consumed)
    } else {
        0
    };

    FsListDirResp {
        err: FsError::Ok,
        epoch: fs_epoch(),
        entries,
        next_cursor,
    }
}

fn handle_stat<S: littlefs2::driver::Storage>(
    fs: &Filesystem<'_, S>,
    req_path: HString<FS_PATH_CAP>,
    expected_epoch: Option<FsEpoch>,
) -> FsStatResp {
    if !epoch_matches(expected_epoch) {
        return fs_stat_err(FsError::EpochMismatch);
    }

    let norm_path = match normalize_req_path(req_path.as_str()) {
        Ok(p) => p,
        Err(err) => return fs_stat_err(err),
    };

    let lfs_path = match to_lfs_path(&norm_path) {
        Ok(p) => p,
        Err(err) => return fs_stat_err(err),
    };

    if path_exists_as_dir(fs, lfs_path.as_path()) {
        return FsStatResp {
            err: FsError::Ok,
            epoch: fs_epoch(),
            kind: FsNodeKind::Dir,
            size_bytes: 0,
        };
    }

    if let Some(size_bytes) = file_size_bytes(fs, lfs_path.as_path()) {
        return FsStatResp {
            err: FsError::Ok,
            epoch: fs_epoch(),
            kind: FsNodeKind::File,
            size_bytes,
        };
    }

    fs_stat_err(FsError::NotFound)
}

fn handle_read_file<S: littlefs2::driver::Storage>(
    fs: &Filesystem<'_, S>,
    req_path: HString<FS_PATH_CAP>,
    offset: u32,
    len: u16,
    expected_epoch: Option<FsEpoch>,
) -> FsReadFileResp {
    if !epoch_matches(expected_epoch) {
        return fs_read_file_err(FsError::EpochMismatch, offset);
    }

    let norm_path = match normalize_req_path(req_path.as_str()) {
        Ok(p) => p,
        Err(err) => return fs_read_file_err(err, offset),
    };

    let lfs_path = match to_lfs_path(&norm_path) {
        Ok(p) => p,
        Err(err) => return fs_read_file_err(err, offset),
    };

    if path_exists_as_dir(fs, lfs_path.as_path()) {
        return fs_read_file_err(FsError::IsDir, offset);
    }

    let Some(size_bytes) = file_size_bytes(fs, lfs_path.as_path()) else {
        return fs_read_file_err(FsError::NotFound, offset);
    };

    if offset > size_bytes {
        return fs_read_file_err(FsError::OffsetOutOfRange, offset);
    }

    let remaining = size_bytes.saturating_sub(offset) as usize;
    let req_len = if len == 0 { FS_MAX_CHUNK } else { len as usize };
    let to_read = core::cmp::min(
        remaining,
        core::cmp::min(req_len, core::cmp::min(FS_MAX_CHUNK, FS_READ_DATA_CAP)),
    );

    if to_read == 0 {
        return FsReadFileResp {
            err: FsError::Ok,
            epoch: fs_epoch(),
            offset,
            data: HVec::new(),
            done: true,
        };
    }

    let mut fetched = 0usize;
    let mut chunk = [0u8; FS_READ_DATA_CAP];

    let read_res = fs.open_file_with_options_and_then(
        |opts| opts.read(true),
        lfs_path.as_path(),
        |file| -> littlefs2::io::Result<()> {
            file.seek(SeekFrom::Start(offset))?;
            fetched = file.read(&mut chunk[..to_read])?;
            Ok(())
        },
    );

    if read_res.is_err() {
        return fs_read_file_err(FsError::Io, offset);
    }

    let mut data: HVec<u8, FS_READ_DATA_CAP> = HVec::new();
    let _ = data.extend_from_slice(&chunk[..fetched]);

    let done =
        (offset as usize).saturating_add(fetched) >= size_bytes as usize || fetched < to_read;

    FsReadFileResp {
        err: FsError::Ok,
        epoch: fs_epoch(),
        offset,
        data,
        done,
    }
}

fn handle_remove<S: littlefs2::driver::Storage>(
    fs: &Filesystem<'_, S>,
    req_path: HString<FS_PATH_CAP>,
    expected_epoch: Option<FsEpoch>,
) -> FsRemoveResp {
    if !epoch_matches(expected_epoch) {
        return fs_remove_err(FsError::EpochMismatch);
    }

    let norm_path = match normalize_req_path(req_path.as_str()) {
        Ok(p) => p,
        Err(err) => return fs_remove_err(err),
    };

    if norm_path.as_str() == "/" {
        return fs_remove_err(FsError::Busy);
    }

    let lfs_path = match to_lfs_path(&norm_path) {
        Ok(p) => p,
        Err(err) => return fs_remove_err(err),
    };

    let existed_as_dir = path_exists_as_dir(fs, lfs_path.as_path());
    let existed_as_file = if existed_as_dir {
        false
    } else {
        path_exists_as_file(fs, lfs_path.as_path())
    };

    if !existed_as_dir && !existed_as_file {
        return fs_remove_err(FsError::NotFound);
    }

    match fs.remove(lfs_path.as_path()) {
        Ok(()) => {
            note_fs_mutation();
            FsRemoveResp {
                err: FsError::Ok,
                epoch: fs_epoch(),
            }
        }
        Err(_) => fs_remove_err(FsError::Busy),
    }
}

fn prepare_log_session<S: Storage>(fs: &Filesystem<S>) -> Option<PathBuf> {
    let log_index = select_next_log_index(fs).ok()?;
    let mut index_buf = itoa::Buffer::new();
    let log_dir = log_dir_path(log_index, &mut index_buf)?;

    embedded_utils::info!("logging task: logging to session dir /log_{}", log_index);

    if fs.exists(log_dir.as_path()) {
        embedded_utils::debug!("logging task: clearing existing session directory");
        if fs.remove_dir_all(log_dir.as_path()).is_err() {
            embedded_utils::fmt::warn!("logging task: failed to clear session directory");
            return None;
        }
    }

    if fs.create_dir(log_dir.as_path()).is_err() {
        embedded_utils::fmt::warn!("logging task: failed to create session directory");
        return None;
    }

    let defmt_file = Path::from_bytes_with_nul(b"defmt.bin\0").ok()?;
    let mut defmt_path = PathBuf::from(log_dir.as_path());
    defmt_path.push(defmt_file);
    if fs.write(defmt_path.as_path(), &[]).is_err() {
        embedded_utils::fmt::warn!("logging task: failed to create defmt.bin");
        return None;
    }

    let build_info_file = Path::from_bytes_with_nul(b"build_info.txt\0").ok()?;
    let mut build_info_path = PathBuf::from(log_dir.as_path());
    build_info_path.push(build_info_file);
    if write_build_info(fs, build_info_path.as_path()).is_err() {
        embedded_utils::fmt::warn!("logging task: failed to write build_info.txt");
        return None;
    }

    note_fs_mutation();
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
        embedded_utils::fmt::warn!("logging task: failed to scan root directory");
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
