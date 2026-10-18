#![allow(dead_code)]
#![allow(clippy::all)]
#![allow(clippy::pedantic)]
#![allow(clippy::doc_markdown)]
#![allow(clippy::needless_raw_string_hashes)]

include!(concat!(env!("OUT_DIR"), "/built.rs"));

use datatypes::status::BuildInformationCommon;
use embassy_sync::lazy_lock::LazyLock;

/// Only the flags the build status LED looks at are filled in, this board does not report build info over CAN yet.
pub static BUILD_INFO: LazyLock<BuildInformationCommon> =
    LazyLock::new(|| BuildInformationCommon {
        is_release: PROFILE == "release",
        debug_defmt_rtt: FEATURES_LOWERCASE.contains(&"debug"),
        is_git_dirty: GIT_DIRTY.unwrap_or(false),
        ..Default::default()
    });
