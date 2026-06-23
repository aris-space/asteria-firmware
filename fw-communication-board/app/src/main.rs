// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::Timer;
use embedded_utils::fmt::*;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32g473_clocks.rs"
    ));
}

use clocks::clocks_config;

mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let config = clocks_config();
    let p = embassy_stm32::init(config);

    let _green = Output::new(p.PB0, Level::Low, Speed::Low);
    let yellow = Output::new(p.PB1, Level::Low, Speed::Low);
    let _red = Output::new(p.PB2, Level::Low, Speed::Low);

    debug!(
        "pkg_name: {}, git_commit_hash_short: {}, git_dirty: {}, profile: {}, features: {}, rustc: {}, target: {}",
        built_info::PKG_NAME,
        built_info::GIT_COMMIT_HASH_SHORT,
        built_info::GIT_DIRTY,
        built_info::PROFILE,
        built_info::FEATURES,
        built_info::RUSTC,
        built_info::TARGET
    );

    spawner.spawn(blink(yellow).unwrap());

    loop {
        info!("Hello, world!");
        Timer::after_secs(1).await;
    }
}

#[embassy_executor::task]
async fn blink(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(900).await;
    }
}
