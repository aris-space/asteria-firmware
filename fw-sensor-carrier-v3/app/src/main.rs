#![no_std]
#![no_main]

use core::future::pending;
mod built;
mod clocks;
mod tasks;
mod resources;
mod macros;

use embassy_executor::{InterruptExecutor, Spawner};

#[allow(unused_imports)]
use panic_reset as _;

#[allow(unused_imports)]
use defmt_rtt as _;


pub static INTERRUPT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::main]
async fn main(level_t_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(clocks::clocks_config());

    let level_0_spawner = interrupt_executor!(TIM2, P6);

    level_0_spawner.must_spawn(tasks::blinky::run());

    loop {
        pending::<()>().await;
    }
}
