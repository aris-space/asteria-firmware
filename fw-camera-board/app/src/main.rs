// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use core::future::pending;

use embassy_executor::Spawner;

mod built;
mod clocks;
mod resources;
// TODO: untested on hw
// mod runcam;
mod signals;
mod startup;
mod tasks;

use defmt_rtt as _;
#[cfg(feature = "debug")]
use panic_probe as _;
#[cfg(not(feature = "debug"))]
use panic_reset as _;

#[embassy_executor::main]
async fn main(thread_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(clocks::clocks_config());
    let board = startup::prepare(resources::split(p));

    startup::spawn_tasks(board, thread_spawner);

    loop {
        pending::<()>().await;
    }
}
