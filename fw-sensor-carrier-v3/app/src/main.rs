// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use core::future::pending;

use embassy_executor::Spawner;

mod built;
mod macros;
mod measurements;
mod params;
mod resources;
mod sensors;
mod signals;
mod startup;
mod storage;
mod tasks;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32h723_clocks.rs"
    ));
}

#[allow(unused_imports)]
use panic_reset as _;

#[embassy_executor::main]
async fn main(thread_spawner: Spawner) -> ! {
    let defmt_consumer = defmt_brtt::init().expect("defmt-brtt init failed");

    let p = embassy_stm32::init(clocks::clocks_config());
    let board = startup::prepare(resources::split(p));
    let level_0_spawner = interrupt_executor!(TIM2, P6);

    startup::spawn_tasks(board, thread_spawner, level_0_spawner, defmt_consumer);

    loop {
        pending::<()>().await;
    }
}
