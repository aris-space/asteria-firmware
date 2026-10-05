//! Build metadata from the `built` crate, and the CAN build-info frame made from it.

#![allow(clippy::all)]
#![allow(clippy::pedantic)]
#![allow(clippy::doc_markdown)]
#![allow(clippy::needless_raw_string_hashes)]

use hermes_can::messages::debug_info::BuildInformationCommon;

include!(concat!(env!("OUT_DIR"), "/built.rs"));

pub fn can_build_information() -> BuildInformationCommon {
    let mut commit_hash = [0; 7];
    if let Some(hash) = GIT_COMMIT_HASH_SHORT {
        let len = hash.len().min(commit_hash.len());
        commit_hash[..len].copy_from_slice(&hash.as_bytes()[..len]);
    }
    BuildInformationCommon {
        unix_timestamp: env!("BUILT_UNIX_TS").parse().unwrap_or(u32::MAX),
        author_initials: *b"LS",
        is_release: PROFILE == "release",
        debug_defmt_rtt: FEATURES_LOWERCASE.contains(&"debug"),
        commit_hash,
        is_git_dirty: GIT_DIRTY.unwrap_or(false),
        can_semver: [
            hermes_can::VERSION_MAJOR,
            hermes_can::VERSION_MINOR,
            hermes_can::VERSION_PATCH,
        ],
    }
}
