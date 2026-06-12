#![no_std]
#![no_main]

use core::future::pending;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

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
mod timing;

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

    // Enable the DWT cycle counter — used for the per-step predict/correct
    // durations logged to the SD card.
    timing::init_dwt();

    let board = startup::prepare(resources::split(p));
    let level_0_spawner = interrupt_executor!(TIM2, P6);
    // Lower-priority executor for state estimation so a long EKF predict
    // cannot starve sensor readouts on level_0.
    let level_1_spawner = interrupt_executor!(TIM3, P7);

    let wait_seconds: usize = 3;
    for i in 0..wait_seconds {
        defmt::info!("Starting in {} seconds...", wait_seconds - i);

        Timer::after(Duration::from_millis(1000)).await;
    }

    startup::spawn_tasks(
        board,
        thread_spawner,
        level_0_spawner,
        level_1_spawner,
        defmt_consumer,
    );

    loop {
        pending::<()>().await;
    }
}
