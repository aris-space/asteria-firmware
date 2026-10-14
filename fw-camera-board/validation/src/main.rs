// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use defmt::println;
use embassy_executor::Spawner;

use defmt_rtt as _;
use panic_probe as _;

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    panic_probe::hard_fault()
}

mod checks;
mod clocks;
mod resources;
// TODO: untested on hw
// mod runcam;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(clocks::clocks_config());

    let r = resources::split(p);

    println!("=== camera-board hardware validation ===");

    let mut led = r.debug_led.setup();
    checks::led(&mut led).await;

    checks::address(r.address).await;

    checks::camera(r.camera_power.setup()).await
}
