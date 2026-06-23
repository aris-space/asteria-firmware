// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![cfg_attr(not(feature = "use-std"), no_std)]

use heapless::{String, Vec};
use postcard_rpc::{TopicDirection, endpoints, topics};
use postcard_schema::Schema;
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Filesystem ICD
// ---------------------------------------------------------------------------

pub const FS_PATH_CAP: usize = 128;
pub const FS_NAME_CAP: usize = 64;
pub const FS_READ_DATA_CAP: usize = 2048;
pub const FS_DIR_PAGE_CAP: usize = 32;
pub const PANIC_MSG_CAP: usize = 64;

pub type FsEpoch = u32;

#[derive(Serialize, Deserialize, Schema, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FsError {
    Ok = 0,
    NotFound,
    NotDir,
    IsDir,
    OffsetOutOfRange,
    Io,
    Busy,
    EpochMismatch,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsInfoReq;

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsInfoResp {
    pub err: FsError,
    pub epoch: FsEpoch,
    pub max_chunk: u16,
    pub max_dir_entries: u16,
    pub uptime_ms: u64,
    pub fs_ready: bool,
    pub current_log_dir: Option<String<FS_PATH_CAP>>,
    pub artifact_timestamp_ms: Option<u64>,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FsNodeKind {
    File = 0,
    Dir = 1,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsDirEntry {
    pub kind: FsNodeKind,
    pub name: String<FS_NAME_CAP>,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsListDirReq {
    pub path: String<FS_PATH_CAP>,
    pub cursor: u32,
    pub max_entries: u16,
    pub expected_epoch: Option<FsEpoch>,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsListDirResp {
    pub err: FsError,
    pub epoch: FsEpoch,
    pub entries: Vec<FsDirEntry, FS_DIR_PAGE_CAP>,
    pub next_cursor: u32,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsStatReq {
    pub path: String<FS_PATH_CAP>,
    pub expected_epoch: Option<FsEpoch>,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsStatResp {
    pub err: FsError,
    pub epoch: FsEpoch,
    pub kind: FsNodeKind,
    pub size_bytes: u32,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsReadFileReq {
    pub path: String<FS_PATH_CAP>,
    pub offset: u32,
    pub len: u16,
    pub expected_epoch: Option<FsEpoch>,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsReadFileResp {
    pub err: FsError,
    pub epoch: FsEpoch,
    pub offset: u32,
    pub data: Vec<u8, FS_READ_DATA_CAP>,
    pub done: bool,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsRemoveReq {
    pub path: String<FS_PATH_CAP>,
    pub expected_epoch: Option<FsEpoch>,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsRemoveResp {
    pub err: FsError,
    pub epoch: FsEpoch,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsEraseStorageReq {
    pub expected_epoch: Option<FsEpoch>,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct FsEraseStorageResp {
    pub err: FsError,
    pub epoch: FsEpoch,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct ResetReq;

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct ResetResp {
    pub ok: bool,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct PanicReq {
    pub message: String<PANIC_MSG_CAP>,
}

#[derive(Serialize, Deserialize, Schema, Debug, Clone)]
pub struct PanicResp {
    pub accepted: bool,
}

// ---------------------------------------------------------------------------
// Endpoint definitions
// ---------------------------------------------------------------------------

endpoints! {
    list = ENDPOINT_LIST;
    | EndpointTy              | RequestTy         | ResponseTy         | Path              |
    | ----------              | ---------         | ----------         | ----              |
    | FsInfoEndpoint          | FsInfoReq         | FsInfoResp         | "fs/info"         |
    | FsListDirEndpoint       | FsListDirReq      | FsListDirResp      | "fs/list_dir"     |
    | FsStatEndpoint          | FsStatReq         | FsStatResp         | "fs/stat"         |
    | FsReadFileEndpoint      | FsReadFileReq     | FsReadFileResp     | "fs/read"         |
    | FsRemoveEndpoint        | FsRemoveReq       | FsRemoveResp       | "fs/remove"       |
    | FsEraseStorageEndpoint  | FsEraseStorageReq | FsEraseStorageResp | "fs/erase_storage" |
    | ResetEndpoint           | ResetReq          | ResetResp          | "sys/reset"       |
    | PanicEndpoint           | PanicReq          | PanicResp          | "sys/panic"       |
}

// ---------------------------------------------------------------------------
// Topic definitions (device → host streaming)
// ---------------------------------------------------------------------------

topics! {
    list = TOPICS_OUT_LIST;
    direction = TopicDirection::ToClient;
    | TopicTy        | MessageTy       | Path               | Cfg |
    | -------        | ---------       | ----               | --- |
}

// ---------------------------------------------------------------------------
// Topic definitions (host → device streaming)
// ---------------------------------------------------------------------------
topics! {
    list = TOPICS_IN_LIST;
    direction = TopicDirection::ToServer;
    | TopicTy | MessageTy | Path | Cfg |
    | ------- | --------- | ---- | --- |
}
