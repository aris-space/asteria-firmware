#![no_std]
#![no_main]

use core::future::pending;

use embassy_executor::Spawner;

mod build_info;
mod built;
mod filters;
mod macros;
mod resources;
mod sensors;
mod signals;
mod startup;
mod tasks;
mod types;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32h723_clocks.rs"
    ));
}

#[allow(unused_imports)]
use defmt_rtt as _;
#[cfg(feature = "debug")]
#[allow(unused_imports)]
use panic_probe as _;
#[cfg(not(feature = "debug"))]
#[allow(unused_imports)]
use panic_reset as _;

#[embassy_executor::main]
async fn main(thread_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(clocks::clocks_config());
    let board = startup::prepare(resources::split(p));
    let level_0_spawner = interrupt_executor!(TIM2, P6);

    startup::spawn_tasks(board, thread_spawner, level_0_spawner);

    loop {
        pending::<()>().await;
    }
}
