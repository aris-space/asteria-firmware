// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

//! Standard board LED tasks, see "Board LEDs" in docs/ARCHITECTURE.md.
//!
//! `active_low` is for LEDs that light when the pin is driven low.

use datatypes::status::BuildInformationCommon;
use embassy_stm32::gpio::{Level, Output};
use embassy_time::Timer;

async fn blink(led: &mut Output<'static>, active_low: bool, on_ms: u64, off_ms: u64) -> ! {
    let on = Level::from(!active_low);
    let off = Level::from(active_low);
    loop {
        led.set_level(on);
        Timer::after_millis(on_ms).await;
        led.set_level(off);
        Timer::after_millis(off_ms).await;
    }
}

/// Green heartbeat LED: 750 ms on, 250 ms off (75% duty).
#[embassy_executor::task]
pub async fn heartbeat(mut led: Output<'static>, active_low: bool) -> ! {
    blink(&mut led, active_low, 750, 250).await
}

/// Red build status LED.
///
/// - clean release build: 1800 ms on, 200 ms off (90% duty)
/// - dirty tree, non-release profile, or defmt over RTT enabled: 125 ms on, 125 ms off
#[embassy_executor::task]
pub async fn build_status(
    mut led: Output<'static>,
    build_info: &'static BuildInformationCommon,
    active_low: bool,
) -> ! {
    let warning_build =
        build_info.is_git_dirty || !build_info.is_release || build_info.debug_defmt_rtt;
    let (on_ms, off_ms) = if warning_build {
        (125, 125)
    } else {
        (1800, 200)
    };
    blink(&mut led, active_low, on_ms, off_ms).await
}
