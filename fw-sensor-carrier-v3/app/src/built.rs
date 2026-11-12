//! Build metadata from the `built` crate, and the CAN build-info frame made from it.

#![allow(clippy::all)]
#![allow(clippy::pedantic)]
#![allow(clippy::doc_markdown)]
#![allow(clippy::needless_raw_string_hashes)]

use core::fmt;

use datatypes::status::BuildInformationCommon;
use embassy_sync::lazy_lock::LazyLock;

include!(concat!(env!("OUT_DIR"), "/built.rs"));

/// The `key=value` lines fw-test-board writes to its `build_info.txt`.
pub fn write_info(out: &mut impl fmt::Write) -> fmt::Result {
    let dirty = GIT_DIRTY.map_or("none", |dirty| if dirty { "true" } else { "false" });
    writeln!(out, "pkg_name={PKG_NAME}")?;
    writeln!(out, "version={PKG_VERSION}")?;
    writeln!(out, "profile={PROFILE}")?;
    writeln!(out, "target={TARGET}")?;
    writeln!(
        out,
        "git_commit_short={}",
        GIT_COMMIT_HASH_SHORT.unwrap_or("none")
    )?;
    writeln!(out, "git_dirty={dirty}")?;
    writeln!(out, "features={FEATURES_LOWERCASE_STR}")?;
    writeln!(out, "rustc={RUSTC}")?;
    writeln!(out, "rustc_version={RUSTC_VERSION}")
}

pub static BUILD_INFO: LazyLock<BuildInformationCommon> = LazyLock::new(|| {
    let mut commit_hash = [0; _];
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
            dp_sensor_carrier::VERSION_MAJOR,
            dp_sensor_carrier::VERSION_MINOR,
            dp_sensor_carrier::VERSION_PATCH,
        ],
    }
});
