// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use core::future::pending;

use embassy_executor::Spawner;

mod built;
mod resources;
mod signals;
mod startup;
mod tasks;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32g473_clocks.rs"
    ));
}

use defmt_rtt as _;
#[cfg(feature = "debug")]
use panic_probe as _;
#[cfg(not(feature = "debug"))]
use panic_reset as _;

#[embassy_executor::main]
async fn main(thread_spawner: Spawner) -> ! {
    // The G4A1 shares the G473 clock tree: 160 MHz SYSCLK from the 16 MHz crystal.
    let p = embassy_stm32::init(clocks::clocks_config());
    let board = startup::prepare(resources::split(p)).await;

    startup::spawn_tasks(board, thread_spawner);

    loop {
        pending::<()>().await;
    }
}
