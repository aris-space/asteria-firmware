use datatypes::status::BuildInformationCommon;
use embassy_sync::lazy_lock::LazyLock;

/// Only the flags the build status LED looks at are filled in, this board does not report build info over CAN yet.
pub static BUILD_INFO: LazyLock<BuildInformationCommon> = LazyLock::new(|| BuildInformationCommon {
    is_release: crate::built::PROFILE == "release",
    debug_defmt_rtt: crate::built::FEATURES_LOWERCASE.contains(&"debug"),
    is_git_dirty: crate::built::GIT_DIRTY.unwrap_or(false),
    ..Default::default()
});
