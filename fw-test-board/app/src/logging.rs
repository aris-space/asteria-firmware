#![allow(clippy::wildcard_imports)]

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use defmt_brtt::DefmtConsumer;
use embassy_sync::blocking_mutex::ThreadModeMutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::Instant;
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
const FS_REQ_CH_CAP: usize = 1;
const FS_EPOCH_INITIAL: u32 = 1;
const LOGGING_DROPPED_WRITE_WARN_EVERY: u32 = 128;
const LOG_DIR_PREFIX: &[u8; 5] = b"/log_";
const LOG_DIR_PATH_MAX_LEN: usize = 16;
const LOG_DIR_NAME_PREFIX: &str = "log_";
const PROTECTED_ROOT_PATH: &str = "/";
const ROOT_DIR_PATH_NUL: &[u8] = b"/\0";
const DEFMT_BIN_FILE_NUL: &[u8] = b"defmt.bin\0";
const BUILD_INFO_FILE_NUL: &[u8] = b"build_info.txt\0";

static LOG_STORAGE: StaticCell<FlashAdapter<'static>> = StaticCell::new();
static LOG_ALLOC: StaticCell<Allocation<FlashAdapter<'static>>> = StaticCell::new();

struct FsState {
    fs: Filesystem<'static, FlashAdapter<'static>>,
    defmt_path: PathBuf,
}

#[derive(Default)]
struct RuntimeState {
    last_session_dir: Option<HString<FS_PATH_CAP>>,
}

static FS_STATE: ThreadModeMutex<RefCell<Option<FsState>>> =
    ThreadModeMutex::new(RefCell::new(None));
static RUNTIME_STATE: ThreadModeMutex<RefCell<RuntimeState>> =
    ThreadModeMutex::new(RefCell::new(RuntimeState {
        last_session_dir: None,
    }));

pub(crate) struct FsMailbox {
    req: Channel<CriticalSectionRawMutex, FsRequest, FS_REQ_CH_CAP>,
    // One response signal per request type keeps responses type-safe
    // and avoids runtime request/response mismatch handling.
    info_resp: Signal<CriticalSectionRawMutex, FsInfoResp>,
    list_dir_resp: Signal<CriticalSectionRawMutex, FsListDirResp>,
    stat_resp: Signal<CriticalSectionRawMutex, FsStatResp>,
    read_file_resp: Signal<CriticalSectionRawMutex, FsReadFileResp>,
    remove_resp: Signal<CriticalSectionRawMutex, FsRemoveResp>,
    erase_storage_resp: Signal<CriticalSectionRawMutex, FsEraseStorageResp>,
    // We allow only one in-flight RPC at a time so each signal can be reused safely.
    call_lock: Mutex<CriticalSectionRawMutex, ()>,
}

impl FsMailbox {
    const fn new() -> Self {
        Self {
            req: Channel::new(),
            info_resp: Signal::new(),
            list_dir_resp: Signal::new(),
            stat_resp: Signal::new(),
            read_file_resp: Signal::new(),
            remove_resp: Signal::new(),
            erase_storage_resp: Signal::new(),
            call_lock: Mutex::new(()),
        }
    }

    /// Send a filesystem request and wait for its typed response.
    pub(crate) async fn request<R: FsMailboxCall>(&self, req: R) -> R::Response {
        let _guard = self.call_lock.lock().await;
        let signal = R::response_signal(self);
        signal.reset();
        self.req.send(req.into_request()).await;
        signal.wait().await
    }

    async fn wait_request(&self) -> FsRequest {
        self.req.receive().await
    }
}

pub(crate) static FS_MAILBOX: FsMailbox = FsMailbox::new();
static FS_EPOCH: AtomicU32 = AtomicU32::new(FS_EPOCH_INITIAL);

pub enum FsRequest {
    Info(FsInfoReq),
    ListDir(FsListDirReq),
    Stat(FsStatReq),
    ReadFile(FsReadFileReq),
    Remove(FsRemoveReq),
    EraseStorage(FsEraseStorageReq),
}

pub(crate) trait FsMailboxCall {
    type Response;

    fn into_request(self) -> FsRequest;
    fn response_signal(mailbox: &FsMailbox) -> &Signal<CriticalSectionRawMutex, Self::Response>;
}

macro_rules! impl_fs_mailbox_call {
    ($req_ty:ty, $resp_ty:ty, $req_variant:ident, $signal_field:ident) => {
        impl FsMailboxCall for $req_ty {
            type Response = $resp_ty;

            fn into_request(self) -> FsRequest {
                FsRequest::$req_variant(self)
            }

            fn response_signal(
                mailbox: &FsMailbox,
            ) -> &Signal<CriticalSectionRawMutex, Self::Response> {
                &mailbox.$signal_field
            }
        }
    };
}

impl_fs_mailbox_call!(FsInfoReq, FsInfoResp, Info, info_resp);
impl_fs_mailbox_call!(FsListDirReq, FsListDirResp, ListDir, list_dir_resp);
impl_fs_mailbox_call!(FsStatReq, FsStatResp, Stat, stat_resp);
impl_fs_mailbox_call!(FsReadFileReq, FsReadFileResp, ReadFile, read_file_resp);
impl_fs_mailbox_call!(FsRemoveReq, FsRemoveResp, Remove, remove_resp);
impl_fs_mailbox_call!(
    FsEraseStorageReq,
    FsEraseStorageResp,
    EraseStorage,
    erase_storage_resp
);

trait FsErrorResponse {
    fn from_err(err: FsError, epoch: FsEpoch) -> Self;
}

fn fs_err<R: FsErrorResponse>(err: FsError) -> R {
    R::from_err(err, fs_epoch())
}

impl FsErrorResponse for FsListDirResp {
    fn from_err(err: FsError, epoch: FsEpoch) -> Self {
        Self {
            err,
            epoch,
            entries: HVec::new(),
            next_cursor: 0,
        }
    }
}

impl FsErrorResponse for FsStatResp {
    fn from_err(err: FsError, epoch: FsEpoch) -> Self {
        Self {
            err,
            epoch,
            kind: FsNodeKind::File,
            size_bytes: 0,
        }
    }
}

impl FsErrorResponse for FsRemoveResp {
    fn from_err(err: FsError, epoch: FsEpoch) -> Self {
        Self { err, epoch }
    }
}

impl FsErrorResponse for FsEraseStorageResp {
    fn from_err(err: FsError, epoch: FsEpoch) -> Self {
        Self { err, epoch }
    }
}

/// Returns the current FS mutation epoch used for optimistic client caching.
pub fn fs_epoch() -> u32 {
    FS_EPOCH.load(Ordering::Relaxed)
}

/// Consumes defmt frames and appends raw bytes into the active session file.
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
                    if dropped_writes.is_multiple_of(LOGGING_DROPPED_WRITE_WARN_EVERY) {
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

/// Owns the littlefs instance and handles filesystem RPC requests sequentially
#[embassy_executor::task]
pub async fn fs_worker_task() -> ! {
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
        match req {
            FsRequest::Info(_req) => {
                FS_MAILBOX.info_resp.signal(handle_fs_info());
            }
            FsRequest::ListDir(req) => {
                let resp = with_fs_req(req, |state, req| {
                    handle_list_dir(
                        &state.fs,
                        req.path.as_str(),
                        req.cursor,
                        req.max_entries,
                        req.expected_epoch,
                    )
                })
                .unwrap_or_else(|_| fs_err(FsError::Io));
                FS_MAILBOX.list_dir_resp.signal(resp);
            }
            FsRequest::Stat(req) => {
                let resp = with_fs_req(req, |state, req| {
                    handle_stat(&state.fs, req.path.as_str(), req.expected_epoch)
                })
                .unwrap_or_else(|_| fs_err(FsError::Io));
                FS_MAILBOX.stat_resp.signal(resp);
            }
            FsRequest::ReadFile(req) => {
                let resp = with_fs_req(req, |state, req| {
                    handle_read_file(
                        &state.fs,
                        req.path.as_str(),
                        req.offset,
                        req.len,
                        req.expected_epoch,
                    )
                })
                .unwrap_or_else(|req| fs_read_file_err(FsError::Io, req.offset));
                FS_MAILBOX.read_file_resp.signal(resp);
            }
            FsRequest::Remove(req) => {
                let resp = with_fs_req(req, |state, req| {
                    handle_remove(&state.fs, req.path.as_str(), req.expected_epoch)
                })
                .unwrap_or_else(|_| fs_err(FsError::Io));
                FS_MAILBOX.remove_resp.signal(resp);
            }
            FsRequest::EraseStorage(_req) => {
                FS_MAILBOX.erase_storage_resp.signal(handle_erase_storage());
            }
        }
    }
}

fn fs_available() -> bool {
    FS_STATE.lock(|cell| cell.borrow().is_some())
}

fn runtime_last_session_dir() -> Option<HString<FS_PATH_CAP>> {
    RUNTIME_STATE.lock(|cell| cell.borrow().last_session_dir.clone())
}

fn runtime_set_last_session_dir(path: &HString<FS_PATH_CAP>) {
    RUNTIME_STATE.lock(|cell| {
        cell.borrow_mut().last_session_dir = Some(path.clone());
    });
}

fn artifact_timestamp_ms() -> Option<u64> {
    crate::built_info::ASTERIA_ARTIFACT_TIMESTAMP_MS?
        .parse::<u64>()
        .ok()
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
    let Ok(fs) = Filesystem::mount(alloc, storage) else {
        embedded_utils::fmt::warn!("fs worker: mount failed after format");
        return None;
    };

    embedded_utils::info!("fs worker: littlefs mounted successfully");

    let Some((defmt_path, current_log_dir)) = prepare_log_session(&fs) else {
        embedded_utils::fmt::warn!("fs worker: session setup failed");
        return None;
    };

    runtime_set_last_session_dir(&current_log_dir);
    Some(FsState { fs, defmt_path })
}

fn handle_erase_storage() -> FsEraseStorageResp {
    FS_STATE.lock(|cell| {
        let mut slot = cell.borrow_mut();
        let Some(state) = slot.take() else {
            return fs_err(FsError::Io);
        };

        let (_alloc, storage) = state.fs.into_inner();

        // Issue a hardware chip-erase command and busy-poll until complete.
        // Takes ~80 s (typ.) / ~400 s (max).  The board reboots immediately
        // after, so littlefs is re-initialised fresh on next boot.
        embedded_utils::info!("fs worker: starting chip erase (~80 s)");
        if unsafe { storage.inner_mut() }
            .blocking_erase_chip()
            .is_err()
        {
            embedded_utils::fmt::warn!("fs worker: chip erase failed");
            return fs_err(FsError::Io);
        }
        embedded_utils::info!("fs worker: chip erase complete");

        FsEraseStorageResp {
            err: FsError::Ok,
            epoch: fs_epoch(),
        }
    })
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

fn fs_read_file_err(err: FsError, offset: u32) -> FsReadFileResp {
    FsReadFileResp {
        err,
        epoch: fs_epoch(),
        offset,
        data: HVec::new(),
        done: true,
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

fn to_lfs_path(path: &HString<FS_PATH_CAP>) -> Result<PathBuf, FsError> {
    PathBuf::try_from(path.as_str()).map_err(|_| FsError::NotFound)
}

fn path_exists_as_file<S: Storage>(fs: &Filesystem<'_, S>, path: &Path) -> bool {
    fs.open_file_with_options_and_then(
        |opts| opts.read(true),
        path,
        |_file| -> littlefs2::io::Result<()> { Ok(()) },
    )
    .is_ok()
}

fn path_exists_as_dir<S: Storage>(fs: &Filesystem<'_, S>, path: &Path) -> bool {
    fs.read_dir_and_then(path, |_dir| -> littlefs2::io::Result<()> { Ok(()) })
        .is_ok()
}

fn file_size_bytes<S: Storage>(fs: &Filesystem<'_, S>, path: &Path) -> Option<u32> {
    let mut size = None;
    let _ = fs.open_file_with_options_and_then(
        |opts| opts.read(true),
        path,
        |file| -> littlefs2::io::Result<()> {
            size = Some(file.seek(SeekFrom::End(0))?);
            Ok(())
        },
    );
    size.map(|n| u32::try_from(n).unwrap_or(u32::MAX))
}

fn handle_fs_info() -> FsInfoResp {
    FsInfoResp {
        err: FsError::Ok,
        epoch: fs_epoch(),
        max_chunk: u16::try_from(FS_MAX_CHUNK).unwrap_or(u16::MAX),
        max_dir_entries: u16::try_from(FS_DIR_PAGE_CAP).unwrap_or(u16::MAX),
        uptime_ms: Instant::now().as_millis(),
        fs_ready: fs_available(),
        current_log_dir: runtime_last_session_dir(),
        artifact_timestamp_ms: artifact_timestamp_ms(),
    }
}

fn handle_list_dir<S: Storage>(
    fs: &Filesystem<'_, S>,
    req_path: &str,
    cursor: u32,
    max_entries: u16,
    expected_epoch: Option<FsEpoch>,
) -> FsListDirResp {
    if !epoch_matches(expected_epoch) {
        return fs_err(FsError::EpochMismatch);
    }

    let norm_path = match normalize_req_path(req_path) {
        Ok(p) => p,
        Err(err) => return fs_err(err),
    };

    let lfs_path = match to_lfs_path(&norm_path) {
        Ok(p) => p,
        Err(err) => return fs_err(err),
    };

    let page_cap = core::cmp::min(usize::from(max_entries.max(1)), FS_DIR_PAGE_CAP);
    let mut entries: HVec<FsDirEntry, FS_DIR_PAGE_CAP> = HVec::new();
    let mut total_entries = 0usize;
    let mut page_scanned = 0usize;
    let cursor_usize = usize::try_from(cursor).unwrap_or(usize::MAX);

    let list_res = fs.read_dir_and_then(lfs_path.as_path(), |dir| -> littlefs2::io::Result<()> {
        for entry in dir {
            let entry = entry?;
            let name_str = entry.file_name().as_str();
            if name_str == "." || name_str == ".." {
                continue;
            }

            if total_entries >= cursor_usize && page_scanned < page_cap {
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
            return fs_err(FsError::NotDir);
        }
        return fs_err(FsError::NotFound);
    }

    let consumed = u32::try_from(page_scanned).unwrap_or(u32::MAX);
    let total_entries_u32 = u32::try_from(total_entries).unwrap_or(u32::MAX);
    let next_cursor = if total_entries_u32 > cursor.saturating_add(consumed) {
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

fn handle_stat<S: Storage>(
    fs: &Filesystem<'_, S>,
    req_path: &str,
    expected_epoch: Option<FsEpoch>,
) -> FsStatResp {
    if !epoch_matches(expected_epoch) {
        return fs_err(FsError::EpochMismatch);
    }

    let norm_path = match normalize_req_path(req_path) {
        Ok(p) => p,
        Err(err) => return fs_err(err),
    };

    let lfs_path = match to_lfs_path(&norm_path) {
        Ok(p) => p,
        Err(err) => return fs_err(err),
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

    fs_err(FsError::NotFound)
}

fn handle_read_file<S: Storage>(
    fs: &Filesystem<'_, S>,
    req_path: &str,
    offset: u32,
    len: u16,
    expected_epoch: Option<FsEpoch>,
) -> FsReadFileResp {
    if !epoch_matches(expected_epoch) {
        return fs_read_file_err(FsError::EpochMismatch, offset);
    }

    let norm_path = match normalize_req_path(req_path) {
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

    let remaining = usize::try_from(size_bytes.saturating_sub(offset)).unwrap_or(usize::MAX);
    let req_len = if len == 0 {
        FS_MAX_CHUNK
    } else {
        usize::from(len)
    };
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

    let offset_usize = usize::try_from(offset).unwrap_or(usize::MAX);
    let size_usize = usize::try_from(size_bytes).unwrap_or(usize::MAX);
    let done = offset_usize.saturating_add(fetched) >= size_usize || fetched < to_read;

    FsReadFileResp {
        err: FsError::Ok,
        epoch: fs_epoch(),
        offset,
        data,
        done,
    }
}

fn handle_remove<S: Storage>(
    fs: &Filesystem<'_, S>,
    req_path: &str,
    expected_epoch: Option<FsEpoch>,
) -> FsRemoveResp {
    if !epoch_matches(expected_epoch) {
        return fs_err(FsError::EpochMismatch);
    }

    let norm_path = match normalize_req_path(req_path) {
        Ok(p) => p,
        Err(err) => return fs_err(err),
    };

    if norm_path.as_str() == PROTECTED_ROOT_PATH {
        return fs_err(FsError::Busy);
    }

    let lfs_path = match to_lfs_path(&norm_path) {
        Ok(p) => p,
        Err(err) => return fs_err(err),
    };

    let existed_as_dir = path_exists_as_dir(fs, lfs_path.as_path());
    let existed_as_file = if existed_as_dir {
        false
    } else {
        path_exists_as_file(fs, lfs_path.as_path())
    };

    if !existed_as_dir && !existed_as_file {
        return fs_err(FsError::NotFound);
    }

    match fs.remove(lfs_path.as_path()) {
        Ok(()) => {
            note_fs_mutation();
            FsRemoveResp {
                err: FsError::Ok,
                epoch: fs_epoch(),
            }
        }
        Err(_) => fs_err(FsError::Busy),
    }
}

fn prepare_log_session<S: Storage>(fs: &Filesystem<S>) -> Option<(PathBuf, HString<FS_PATH_CAP>)> {
    let log_index = select_next_log_index(fs).ok()?;
    let mut index_buf = itoa::Buffer::new();
    let log_dir = log_dir_path(log_index, &mut index_buf)?;
    let mut session_dir = HString::<FS_PATH_CAP>::new();
    session_dir.push('/').ok()?;
    session_dir.push_str(LOG_DIR_NAME_PREFIX).ok()?;
    session_dir.push_str(index_buf.format(log_index)).ok()?;

    embedded_utils::info!("fs worker: logging to session dir /log_{}", log_index);

    if fs.exists(log_dir.as_path()) {
        embedded_utils::debug!("fs worker: clearing existing session directory");
        if fs.remove_dir_all(log_dir.as_path()).is_err() {
            embedded_utils::fmt::warn!("fs worker: failed to clear session directory");
            return None;
        }
    }

    if fs.create_dir(log_dir.as_path()).is_err() {
        embedded_utils::fmt::warn!("fs worker: failed to create session directory");
        return None;
    }

    let defmt_file = Path::from_bytes_with_nul(DEFMT_BIN_FILE_NUL).ok()?;
    let mut defmt_path = PathBuf::from(log_dir.as_path());
    defmt_path.push(defmt_file);
    if fs.write(defmt_path.as_path(), &[]).is_err() {
        embedded_utils::fmt::warn!("fs worker: failed to create defmt.bin");
        return None;
    }

    let build_info_file = Path::from_bytes_with_nul(BUILD_INFO_FILE_NUL).ok()?;
    let mut build_info_path = PathBuf::from(log_dir.as_path());
    build_info_path.push(build_info_file);
    if write_build_info(fs, build_info_path.as_path()).is_err() {
        embedded_utils::fmt::warn!("fs worker: failed to write build_info.txt");
        return None;
    }

    note_fs_mutation();
    Some((defmt_path, session_dir))
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
    let root = Path::from_bytes_with_nul(ROOT_DIR_PATH_NUL).map_err(|_| ())?;
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
        embedded_utils::fmt::warn!("fs worker: failed to scan root directory");
        return Err(());
    }

    Ok(max_index.map_or(0, |n| n.saturating_add(1)))
}

fn parse_log_dir_index(name: &str) -> Option<u32> {
    let digits = name.strip_prefix(LOG_DIR_NAME_PREFIX)?;
    if digits.is_empty() {
        return None;
    }

    let mut value: u32 = 0;
    for ch in digits.as_bytes() {
        if !ch.is_ascii_digit() {
            return None;
        }
        value = value.checked_mul(10)?;
        value = value.checked_add(u32::from(ch - b'0'))?;
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
