use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::time::Duration;

use anyhow::{Result, anyhow, bail};
use async_trait::async_trait;
use postcard_rpc::host_client::HostClient;
use postcard_rpc::standard_icd::WireError;
use tokio::time::timeout;
use wire_types::{
    FsDirEntry, FsEraseStorageEndpoint, FsEraseStorageReq, FsEraseStorageResp, FsError,
    FsInfoEndpoint, FsInfoReq, FsInfoResp, FsListDirEndpoint, FsListDirReq, FsListDirResp,
    FsNodeKind, FsReadFileEndpoint, FsReadFileReq, FsReadFileResp, FsRemoveEndpoint, FsRemoveReq,
    FsRemoveResp, FsStatEndpoint, FsStatReq, FsStatResp, PanicEndpoint, PanicReq, PanicResp,
    ResetEndpoint, ResetReq, ResetResp, PANIC_MSG_CAP,
};

const RPC_TIMEOUT: Duration = Duration::from_secs(5);
const ERASE_STORAGE_TIMEOUT: Duration = Duration::from_secs(300);

#[derive(Debug, Clone, Eq, PartialEq)]
pub struct DirEntry {
    pub kind: FsNodeKind,
    pub name: String,
}

#[derive(Debug, Clone, Eq, PartialEq)]
pub struct FsNodeMeta {
    pub kind: FsNodeKind,
    pub size_bytes: u32,
    pub epoch: u32,
}

#[derive(Debug, Clone)]
pub struct StorageReport {
    pub epoch: u32,
    pub max_chunk: u16,
    pub max_dir_entries: u16,
}

#[async_trait]
pub trait RpcBackend {
    async fn fs_info(&self) -> Result<FsInfoResp>;
    async fn fs_list_dir_page(
        &self,
        path: &str,
        cursor: u32,
        max_entries: u16,
        expected_epoch: Option<u32>,
    ) -> Result<FsListDirResp>;
    async fn fs_stat(&self, path: &str, expected_epoch: Option<u32>) -> Result<FsStatResp>;
    async fn fs_read_file_chunk(
        &self,
        path: &str,
        offset: u32,
        len: u16,
        expected_epoch: Option<u32>,
    ) -> Result<FsReadFileResp>;
    async fn fs_remove_path(&self, path: &str, expected_epoch: Option<u32>) -> Result<FsRemoveResp>;
    async fn fs_erase_storage(&self, expected_epoch: Option<u32>) -> Result<FsEraseStorageResp>;
    async fn reset_board(&self) -> Result<ResetResp>;
    async fn panic_board(&self, message: &str) -> Result<PanicResp>;
}

#[async_trait]
impl RpcBackend for HostClient<WireError> {
    async fn fs_info(&self) -> Result<FsInfoResp> {
        timeout(RPC_TIMEOUT, self.send_resp::<FsInfoEndpoint>(&FsInfoReq))
            .await
            .map_err(|_| anyhow!("rpc fs/info timed out after {}s", RPC_TIMEOUT.as_secs()))?
            .map_err(|e| anyhow!("rpc fs/info failed: {e:?}"))
    }

    async fn fs_list_dir_page(
        &self,
        path: &str,
        cursor: u32,
        max_entries: u16,
        expected_epoch: Option<u32>,
    ) -> Result<FsListDirResp> {
        let src = if path.is_empty() { "/" } else { path };
        let wire_path = src
            .try_into()
            .map_err(|_| anyhow!("remote path exceeds FS_PATH_CAP: {src}"))?;
        let req = FsListDirReq {
            path: wire_path,
            cursor,
            max_entries,
            expected_epoch,
        };
        timeout(RPC_TIMEOUT, self.send_resp::<FsListDirEndpoint>(&req))
            .await
            .map_err(|_| anyhow!("rpc fs/list_dir timed out for path {path}"))?
            .map_err(|e| anyhow!("rpc fs/list_dir failed for path {path}: {e:?}"))
    }

    async fn fs_stat(&self, path: &str, expected_epoch: Option<u32>) -> Result<FsStatResp> {
        let src = if path.is_empty() { "/" } else { path };
        let wire_path = src
            .try_into()
            .map_err(|_| anyhow!("remote path exceeds FS_PATH_CAP: {src}"))?;
        let req = FsStatReq {
            path: wire_path,
            expected_epoch,
        };
        timeout(RPC_TIMEOUT, self.send_resp::<FsStatEndpoint>(&req))
            .await
            .map_err(|_| anyhow!("rpc fs/stat timed out for path {path}"))?
            .map_err(|e| anyhow!("rpc fs/stat failed for path {path}: {e:?}"))
    }

    async fn fs_read_file_chunk(
        &self,
        path: &str,
        offset: u32,
        len: u16,
        expected_epoch: Option<u32>,
    ) -> Result<FsReadFileResp> {
        let src = if path.is_empty() { "/" } else { path };
        let wire_path = src
            .try_into()
            .map_err(|_| anyhow!("remote path exceeds FS_PATH_CAP: {src}"))?;
        let req = FsReadFileReq {
            path: wire_path,
            offset,
            len,
            expected_epoch,
        };
        timeout(RPC_TIMEOUT, self.send_resp::<FsReadFileEndpoint>(&req))
            .await
            .map_err(|_| anyhow!("rpc fs/read timed out for path {path} offset {offset}"))?
            .map_err(|e| anyhow!("rpc fs/read failed for path {path} offset {offset}: {e:?}"))
    }

    async fn fs_remove_path(&self, path: &str, expected_epoch: Option<u32>) -> Result<FsRemoveResp> {
        let src = if path.is_empty() { "/" } else { path };
        let wire_path = src
            .try_into()
            .map_err(|_| anyhow!("remote path exceeds FS_PATH_CAP: {src}"))?;
        let req = FsRemoveReq {
            path: wire_path,
            expected_epoch,
        };
        timeout(RPC_TIMEOUT, self.send_resp::<FsRemoveEndpoint>(&req))
            .await
            .map_err(|_| anyhow!("rpc fs/remove timed out for path {path}"))?
            .map_err(|e| anyhow!("rpc fs/remove failed for path {path}: {e:?}"))
    }

    async fn fs_erase_storage(&self, expected_epoch: Option<u32>) -> Result<FsEraseStorageResp> {
        let req = FsEraseStorageReq { expected_epoch };
        timeout(
            ERASE_STORAGE_TIMEOUT,
            self.send_resp::<FsEraseStorageEndpoint>(&req),
        )
            .await
            .map_err(|_| {
                anyhow!(
                    "rpc fs/erase_storage timed out after {}s",
                    ERASE_STORAGE_TIMEOUT.as_secs()
                )
            })?
            .map_err(|e| anyhow!("rpc fs/erase_storage failed: {e:?}"))
    }

    async fn reset_board(&self) -> Result<ResetResp> {
        timeout(RPC_TIMEOUT, self.send_resp::<ResetEndpoint>(&ResetReq))
            .await
            .map_err(|_| anyhow!("rpc sys/reset timed out after {}s", RPC_TIMEOUT.as_secs()))?
            .map_err(|e| anyhow!("rpc sys/reset failed: {e:?}"))
    }

    async fn panic_board(&self, message: &str) -> Result<PanicResp> {
        let wire_message = message.try_into().map_err(|_| {
            anyhow!(
                "panic message too long (max {} bytes): {}",
                PANIC_MSG_CAP,
                message
            )
        })?;
        let req = PanicReq {
            message: wire_message,
        };
        timeout(RPC_TIMEOUT, self.send_resp::<PanicEndpoint>(&req))
            .await
            .map_err(|_| anyhow!("rpc sys/panic timed out after {}s", RPC_TIMEOUT.as_secs()))?
            .map_err(|e| anyhow!("rpc sys/panic failed: {e:?}"))
    }
}

fn map_fs_error(op: &str, path: &str, err: FsError) -> anyhow::Error {
    match err {
        FsError::Ok => anyhow!("{op} returned unexpected ok sentinel for {path}"),
        FsError::NotFound => anyhow!("path not found: {path}"),
        FsError::NotDir => anyhow!("not a directory: {path}"),
        FsError::IsDir => anyhow!("is a directory: {path}"),
        FsError::OffsetOutOfRange => anyhow!("offset out of range for {path}"),
        FsError::Io => anyhow!("filesystem I/O error during {op} for {path}"),
        FsError::Busy => anyhow!("path is busy/protected during {op}: {path}"),
        FsError::EpochMismatch => anyhow!("filesystem changed during {op} for {path}"),
    }
}

pub async fn fs_info<B: RpcBackend + Sync>(backend: &B) -> Result<FsInfoResp> {
    let resp = backend.fs_info().await?;
    if resp.err != FsError::Ok {
        return Err(map_fs_error("fs/info", "/", resp.err));
    }
    Ok(resp)
}

pub async fn stat_path<B: RpcBackend + Sync>(backend: &B, path: &str) -> Result<FsNodeMeta> {
    let resp = backend.fs_stat(path, None).await?;
    if resp.err != FsError::Ok {
        return Err(map_fs_error("fs/stat", path, resp.err));
    }
    Ok(FsNodeMeta {
        kind: resp.kind,
        size_bytes: resp.size_bytes,
        epoch: resp.epoch,
    })
}

pub async fn remove_path<B: RpcBackend + Sync>(backend: &B, path: &str) -> Result<u32> {
    let resp = backend.fs_remove_path(path, None).await?;
    if resp.err != FsError::Ok {
        return Err(map_fs_error("fs/remove", path, resp.err));
    }
    Ok(resp.epoch)
}

pub async fn erase_storage<B: RpcBackend + Sync>(backend: &B) -> Result<u32> {
    let resp = backend.fs_erase_storage(None).await?;
    if resp.err != FsError::Ok {
        return Err(map_fs_error("fs/erase_storage", "/", resp.err));
    }
    Ok(resp.epoch)
}

pub async fn reset_board<B: RpcBackend + Sync>(backend: &B) -> Result<()> {
    let resp = backend.reset_board().await?;
    if !resp.ok {
        bail!("device rejected reset request");
    }
    Ok(())
}

pub async fn panic_board<B: RpcBackend + Sync>(backend: &B, message: &str) -> Result<()> {
    let resp = backend.panic_board(message).await?;
    if !resp.accepted {
        bail!("device rejected panic request");
    }
    Ok(())
}

pub async fn list_dir_entries<B: RpcBackend + Sync>(
    backend: &B,
    path: &str,
) -> Result<Vec<DirEntry>> {
    let info = fs_info(backend).await?;
    let mut cursor = 0u32;
    let mut out: Vec<DirEntry> = Vec::new();

    loop {
        let resp = backend
            .fs_list_dir_page(path, cursor, info.max_dir_entries.max(1), None)
            .await?;

        match resp.err {
            FsError::Ok => {
                for FsDirEntry { kind, name } in resp.entries.into_iter() {
                    let name = name.to_string();
                    if name == "." || name == ".." {
                        continue;
                    }
                    out.push(DirEntry {
                        kind,
                        name,
                    });
                }
                if resp.next_cursor == 0 {
                    sort_dir_entries(&mut out);
                    return Ok(out);
                }
                cursor = resp.next_cursor;
            }
            other => {
                return Err(map_fs_error("fs/list_dir", path, other));
            }
        }
    }
}

fn sort_dir_entries(entries: &mut [DirEntry]) {
    entries.sort_by(|a, b| match (a.kind, b.kind) {
        (FsNodeKind::Dir, FsNodeKind::File) => core::cmp::Ordering::Less,
        (FsNodeKind::File, FsNodeKind::Dir) => core::cmp::Ordering::Greater,
        _ => natural_cmp_ascii(&a.name, &b.name),
    });
}

fn natural_cmp_ascii(a: &str, b: &str) -> core::cmp::Ordering {
    let mut ia = 0usize;
    let mut ib = 0usize;
    let ab = a.as_bytes();
    let bb = b.as_bytes();

    while ia < ab.len() && ib < bb.len() {
        let ca = ab[ia];
        let cb = bb[ib];
        let a_is_digit = ca.is_ascii_digit();
        let b_is_digit = cb.is_ascii_digit();

        if a_is_digit && b_is_digit {
            let sa = ia;
            let sb = ib;

            while ia < ab.len() && ab[ia].is_ascii_digit() {
                ia += 1;
            }
            while ib < bb.len() && bb[ib].is_ascii_digit() {
                ib += 1;
            }

            let mut na = sa;
            let mut nb = sb;
            while na < ia && ab[na] == b'0' {
                na += 1;
            }
            while nb < ib && bb[nb] == b'0' {
                nb += 1;
            }

            let la = ia.saturating_sub(na);
            let lb = ib.saturating_sub(nb);
            if la != lb {
                return la.cmp(&lb);
            }

            if na < ia && nb < ib {
                let ord = ab[na..ia].cmp(&bb[nb..ib]);
                if ord != core::cmp::Ordering::Equal {
                    return ord;
                }
            }

            let ord = (ia - sa).cmp(&(ib - sb));
            if ord != core::cmp::Ordering::Equal {
                return ord;
            }
        } else {
            let oa = ca.to_ascii_lowercase();
            let ob = cb.to_ascii_lowercase();
            if oa != ob {
                return oa.cmp(&ob);
            }
            ia += 1;
            ib += 1;
        }
    }

    ab.len().cmp(&bb.len())
}

pub async fn read_file_bytes<B: RpcBackend + Sync>(backend: &B, path: &str) -> Result<Vec<u8>> {
    let info = fs_info(backend).await?;
    let chunk_len = info.max_chunk.max(1);
    let stat = stat_path(backend, path).await?;
    if stat.kind != FsNodeKind::File {
        bail!("is a directory: {path}");
    }
    let target_size = stat.size_bytes;
    let mut offset = 0u32;
    let mut out = Vec::with_capacity(target_size as usize);

    while offset < target_size {
        let remaining = target_size.saturating_sub(offset);
        let req_len = core::cmp::min(chunk_len as u32, remaining) as u16;
        let resp = backend
            .fs_read_file_chunk(path, offset, req_len.max(1), None)
            .await?;

        match resp.err {
            FsError::Ok => {
                if resp.offset != offset {
                    bail!(
                        "rpc fs/read returned unexpected offset: expected {offset}, got {}",
                        resp.offset
                    );
                }
                let chunk = resp.data.as_slice();
                if chunk.is_empty() {
                    bail!("rpc fs/read returned empty data before reaching snapshot size at offset {offset}");
                }
                out.extend_from_slice(chunk);
                offset = offset
                    .checked_add(chunk.len() as u32)
                    .ok_or_else(|| anyhow!("offset overflow while reading {path}"))?;
                if offset > target_size {
                    bail!("rpc fs/read returned more bytes than requested snapshot size for {path}");
                }
            }
            other => return Err(map_fs_error("fs/read", path, other)),
        }
    }

    Ok(out)
}

pub async fn pull_file_to_path_with_progress<B, F>(
    backend: &B,
    path: &str,
    local_path: &Path,
    mut on_progress: F,
) -> Result<usize>
where
    B: RpcBackend + Sync,
    F: FnMut(usize, usize),
{
    if let Some(parent) = local_path.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent)?;
    }

    let info = fs_info(backend).await?;
    let chunk_len = info.max_chunk.max(1);
    let stat = stat_path(backend, path).await?;
    if stat.kind != FsNodeKind::File {
        bail!("is a directory: {path}");
    }
    let target_size = stat.size_bytes;
    let file = File::create(local_path)?;
    let mut writer = BufWriter::with_capacity(128 * 1024, file);
    let mut offset = 0u32;
    let mut total = 0usize;
    on_progress(0, target_size as usize);

    while offset < target_size {
        let remaining = target_size.saturating_sub(offset);
        let req_len = core::cmp::min(chunk_len as u32, remaining) as u16;
        let resp = backend
            .fs_read_file_chunk(path, offset, req_len.max(1), None)
            .await?;

        match resp.err {
            FsError::Ok => {
                if resp.offset != offset {
                    bail!(
                        "rpc fs/read returned unexpected offset: expected {offset}, got {}",
                        resp.offset
                    );
                }
                let chunk = resp.data.as_slice();
                if chunk.is_empty() {
                    bail!("rpc fs/read returned empty data before reaching snapshot size at offset {offset}");
                }
                writer.write_all(chunk)?;
                total = total.saturating_add(chunk.len());
                offset = offset
                    .checked_add(chunk.len() as u32)
                    .ok_or_else(|| anyhow!("offset overflow while pulling {path}"))?;
                on_progress(total, target_size as usize);
                if offset > target_size {
                    bail!("rpc fs/read returned more bytes than requested snapshot size for {path}");
                }
            }
            other => return Err(map_fs_error("fs/read", path, other)),
        }
    }

    writer.flush()?;
    Ok(total)
}

pub async fn build_storage_report<B: RpcBackend + Sync>(backend: &B) -> Result<StorageReport> {
    let info = fs_info(backend).await?;
    Ok(StorageReport {
        epoch: info.epoch,
        max_chunk: info.max_chunk,
        max_dir_entries: info.max_dir_entries,
    })
}

pub fn format_storage_report(report: &StorageReport) -> String {
    format!(
        "Filesystem info:\n  Epoch: {}\n  Max chunk: {} bytes\n  Max dir entries/page: {}",
        report.epoch, report.max_chunk, report.max_dir_entries
    )
}
