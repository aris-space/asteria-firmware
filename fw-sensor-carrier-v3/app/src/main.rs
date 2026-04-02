#![no_std]
#![no_main]

use core::future::pending;
mod built;
mod clocks;
mod tasks;

use embassy_executor::{InterruptExecutor, Spawner};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};

#[allow(unused_imports)]
use panic_reset as _;

#[allow(unused_imports)]
use defmt_rtt as _;


pub static INTERRUPT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[embassy_executor::main]
async fn main(blocking_executor: Spawner) -> ! {
    let p = embassy_stm32::init(clocks::clocks_config());

    interrupt::TIM2.set_priority(Priority::P1);
    let executor = INTERRUPT_EXECUTOR.start(interrupt::TIM2);

    executor.must_spawn(tasks::blinky::run());


    loop {
        pending::<()>().await;
    }
}
