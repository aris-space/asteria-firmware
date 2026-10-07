use datatypes::status::BuildInformationCommon;
use embassy_stm32::gpio::Output;
use embassy_sync::lazy_lock::LazyLock;
use embassy_time::Timer;

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

pub(crate) static BUILD_INFO: LazyLock<BuildInformationCommon> = LazyLock::new(|| {
    const TIMESTAMP: u64 = parse_unix_timestamp(env!("BUILT_UNIX_TS"));
    let commit_hash = parse_commit_hash(built::GIT_COMMIT_HASH_SHORT);
    let is_release = built::PROFILE == "release";
    let debug_defmt_rtt = built::FEATURES.contains(&"defmt");
    let is_git_dirty = built::GIT_DIRTY.unwrap_or(false);
    #[allow(clippy::char_lit_as_u8)] // good as long as it is ascii
    let author_initials = ['C' as u8, 'B' as u8];
    let can_semver = [
        dp_recovery_board::VERSION_MAJOR,
        dp_recovery_board::VERSION_MINOR,
        dp_recovery_board::VERSION_PATCH,
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

/// indication that async is working correctly, hopefully.
/// 4 Hz if something is or 0.5Hz with high duty cycle otherwise.
#[embassy_executor::task]
pub async fn build_status_blinky(mut led: Output<'static>) {
    let build_info = crate::build_info::BUILD_INFO.get();
    let warning_build =
        build_info.is_git_dirty || !build_info.is_release || build_info.debug_defmt_rtt;
    let (on_ms, off_ms) = if warning_build {
        (125, 125)
    } else {
        (200, 1800)
    };

    loop {
        led.set_low();
        Timer::after_millis(on_ms).await;
        led.set_high();
        Timer::after_millis(off_ms).await;
    }
}
