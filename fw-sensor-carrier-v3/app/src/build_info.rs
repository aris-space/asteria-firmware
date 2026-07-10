use embassy_sync::lazy_lock::LazyLock;

use crate::built;

const fn parse_unix_timestamp(s: &str) -> u64 {
    match u64::from_str_radix(s, 10) {
        Ok(ts) => ts,
        Err(_) => panic!("Invalid UNIX timestamp from BUILT_UNIX_TS"),
    }
}

const fn parse_commit_hash(s: Option<&str>) -> [u8; 7] {
    let mut hash = [0; 7];
    if let Some(s) = s {
        let bytes = s.as_bytes();
        if bytes.len() >= 7 {
            let mut i = 0;
            while i < 7 {
                hash[i] = bytes[i];
                i += 1;
            }
        }
    }
    hash
}

#[derive(Clone, Copy, Debug)]
pub struct BuildInfo {
    pub unix_timestamp: u32,
    pub author_initials: [u8; 2],
    pub is_release: bool,
    pub debug_defmt_rtt: bool,
    pub commit_hash: [u8; 7],
    pub is_git_dirty: bool,
}

pub static BUILD_INFO: LazyLock<BuildInfo> = LazyLock::new(|| {
    const TIMESTAMP: u64 = parse_unix_timestamp(env!("BUILT_UNIX_TS"));
    let commit_hash = parse_commit_hash(built::GIT_COMMIT_HASH_SHORT);
    let is_release = built::PROFILE == "release";
    let debug_defmt_rtt = built::FEATURES.contains(&"debug");
    let is_git_dirty = built::GIT_DIRTY.unwrap_or(false);

    BuildInfo {
        unix_timestamp: u32::try_from(TIMESTAMP).unwrap_or(u32::MAX),
        author_initials: [b'L', b'S'],
        is_release,
        debug_defmt_rtt,
        commit_hash,
        is_git_dirty,
    }
});
