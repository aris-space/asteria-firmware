use serde::Serialize;

pub(crate) const DEFMT_CHUNK_SIZE: usize = 512;
pub(crate) const LOG_RECORD_BUFFER_SIZE: usize = 1024;

#[derive(Serialize)]
pub(crate) struct SessionMeta<'a> {
    pub session_id: u32,
    pub firmware_name: &'a str,
    pub profile: &'a str,
    pub target: &'a str,
    pub git_short_hash: Option<&'a str>,
    pub git_dirty: Option<bool>,
    pub features: &'a str,
}

#[derive(Serialize)]
pub(crate) enum LogRecord<'a> {
    SessionStart(SessionMeta<'a>),
    DefmtChunk { session_id: u32, bytes: &'a [u8] },
}

pub(crate) fn session_meta(session_id: u32) -> SessionMeta<'static> {
    use crate::built;

    SessionMeta {
        session_id,
        firmware_name: built::PKG_NAME,
        profile: built::PROFILE,
        target: built::TARGET,
        git_short_hash: built::GIT_COMMIT_HASH_SHORT,
        git_dirty: built::GIT_DIRTY,
        features: built::FEATURES_LOWERCASE_STR,
    }
}
