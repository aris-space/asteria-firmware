// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![no_main]

use core::future::pending;

use embassy_executor::Spawner;

mod built;
mod calibration;
mod macros;
mod resources;
mod sensors;
mod signals;
mod startup;
mod storage;
mod tasks;
mod types;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32h723_clocks.rs"
    ));
}

use defmt_rtt as _;
#[cfg(feature = "debug")]
use panic_probe as _;
#[cfg(not(feature = "debug"))]
use panic_reset as _;

#[embassy_executor::main]
async fn main(thread_spawner: Spawner) -> ! {
    // Code runs from flash, which is several times slower without the instruction
    // cache. The data cache stays off: DMA buffers sit in cacheable SRAM and the
    // HAL does no cache maintenance for them.
    let mut core = cortex_m::Peripherals::take().expect("core peripherals");
    core.SCB.enable_icache();

    let mut config = clocks::clocks_config();
    {
        use embassy_stm32::rcc::*;
        // USB FS needs a 48 MHz kernel clock; HSI48 trimmed off USB SOF (CRS).
        config.rcc.hsi48 = Some(Hsi48Config {
            sync_from_usb: true,
        });
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
    }
    let p = embassy_stm32::init(config);
    // Sensor readouts, highest priority so sampling and timestamps never wait.
    let level_0_spawner = interrupt_executor!(TIM2, P6);
    // Estimation, CAN, and LEDs, above the thread-mode tasks (SD card, console),
    // whose SD card writes busy-wait while the card programs.
    let level_1_spawner = interrupt_executor!(TIM3, P7);
    let board = startup::prepare(resources::split(p), level_1_spawner).await;

    startup::spawn_tasks(board, thread_spawner, level_0_spawner, level_1_spawner).await;

    loop {
        pending::<()>().await;
    }
}
