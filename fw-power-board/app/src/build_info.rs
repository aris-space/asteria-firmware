use embassy_sync::lazy_lock::LazyLock;
use datatypes::status::BuildInformationCommon;

pub mod built {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

const fn parse_unix_timestamp(s: &str) -> u64 {
    match u64::from_str_radix(s, 10) {
        Ok(ts) => ts,
        Err(_) => {
            panic!("Invalid UNIX timestamp in built.rs or not set. Expected a valid u64 string.");
        }
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

pub static BUILD_INFO: LazyLock<BuildInformationCommon> = LazyLock::new(|| {
    const TIMESTAMP: u64 = parse_unix_timestamp(env!("BUILT_UNIX_TS"));
    let commit_hash = parse_commit_hash(built::GIT_COMMIT_HASH_SHORT);
    let is_release = built::PROFILE == "release";
    let debug_defmt_rtt = built::FEATURES_LOWERCASE.contains(&"defmt");
    let is_git_dirty = built::GIT_DIRTY.unwrap_or(false);
    let author_initials = [0, 0]; // todo placeholder
    let can_semver = [
        dp_backplane::VERSION_MAJOR,
        dp_backplane::VERSION_MINOR,
        dp_backplane::VERSION_PATCH,
    ];

    BuildInformationCommon {
        unix_timestamp: u32::try_from(TIMESTAMP).unwrap_or(u32::MAX),
        author_initials,
        is_release,
        debug_defmt_rtt,
        commit_hash,
        is_git_dirty,
        can_semver,
    }
});
