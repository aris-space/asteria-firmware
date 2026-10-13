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
mod resources;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32g473_clocks.rs"
    ));
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(clocks::clocks_config());

    let r = resources::split(p);

    println!("=== camera-board hardware validation ===");

    // Visual first; the operator confirms it by eye.
    let mut led = r.debug_led.setup();
    checks::led(&mut led).await;

    let mut all_passed = true;

    checks::address(r.address).await;

    let camera_power = r.camera_power.setup();
    let camera_uart = r.camera_uart.setup();
    all_passed &= checks::camera(camera_power, camera_uart).await;

    let verdict = if all_passed {
        "=== CORE CHECKS PASSED ==="
    } else {
        "=== SOME CHECKS FAILED (scroll up for details) ==="
    };
    println!("{}", verdict);

    checks::announce(&mut led, all_passed).await
}
