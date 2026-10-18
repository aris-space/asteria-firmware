use embassy_sync::lazy_lock::LazyLock;
use hermes_can::messages::debug_info::BuildInformationCommon;

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
    let debug_defmt_rtt = built::FEATURES.contains(&"defmt");
    let is_git_dirty = built::GIT_DIRTY.unwrap_or(false);
    let author_initials = [b'L', b'S'];
    let can_semver = [
        hermes_can::VERSION_MAJOR,
        hermes_can::VERSION_MINOR,
        hermes_can::VERSION_PATCH,
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


/// `BUILD_INFO` in the type `embedded_utils::led::build_status` expects. This board
/// still sends the legacy hermes-can type over CAN.
pub static LED_BUILD_INFO: LazyLock<datatypes::status::BuildInformationCommon> =
    LazyLock::new(|| {
        let info = BUILD_INFO.get();
        datatypes::status::BuildInformationCommon {
            is_release: info.is_release,
            debug_defmt_rtt: info.debug_defmt_rtt,
            is_git_dirty: info.is_git_dirty,
            ..Default::default()
        }
    });
