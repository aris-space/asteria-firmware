#![no_std]
#![no_main]

use core::future::pending;

mod board;
mod logging;
mod usb_rpc;

use embassy_executor::Spawner;
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::{InterruptExt, Priority};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32g473_clocks.rs"
    ));
}

use clocks::clocks_config;

mod built_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
    pub const ASTERIA_ARTIFACT_TIMESTAMP_MS: Option<&str> =
        option_env!("ASTERIA_ARTIFACT_TIMESTAMP_MS");
}

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use panic_probe as _;

use crate::board::Board;
use embedded_utils::fmt::*;

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

#[embassy_executor::main]
async fn main(blocking_executor: Spawner) -> ! {
    let config = clocks_config();
    let p = embassy_stm32::init(config);
    board::BOARD
        .init(Mutex::new(Board::new(p)))
        .expect("failed to initialize board");

    interrupt::TIM2.set_priority(Priority::P1);
    let async_executor = board::INTERRUPT_EXECUTOR.start(interrupt::TIM2);

    blocking_executor.spawn(logging::logging_task().expect("failed to create logging task"));
    blocking_executor.spawn(logging::fs_worker().expect("failed to create fs_worker task"));

    #[cfg(feature = "log-stress")]
    blocking_executor.spawn(defmt_stress_task().expect("failed to create defmt_stress_task"));

    async_executor.spawn(blink_yellow().expect("failed to create blink_yellow task"));
    async_executor.spawn(usb_rpc::usb_rpc_task().expect("failed to create usb_rpc task"));

    loop {
        pending::<()>().await;
    }
}

#[embassy_executor::task]
async fn blink_yellow() -> ! {
    info!("blink task: startup");
    let mut yellow = {
        let mut board = board::BOARD
            .try_get()
            .expect("BOARD not initialized")
            .try_lock()
            .expect("BOARD mutex busy while taking yellow LED");

        board.yellow.take().expect("yellow LED not available")
    };
    info!("blink task: acquired all resources");

    loop {
        yellow.set_high();
        Timer::after(Duration::from_millis(500)).await;
        yellow.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[cfg(feature = "log-stress")]
#[embassy_executor::task]
async fn defmt_stress_task() -> ! {
    info!("defmt stress task: startup");
    info!("defmt stress task: acquired all resources");

    const STRESS_PAYLOAD_BYTES: usize = 96;
    const STRESS_PERIOD_MS: u64 = 8;

    let mut seq: u32 = 0;
    let mut payload = [0u8; STRESS_PAYLOAD_BYTES];

    loop {
        let mut x = seq ^ 0xD00D_BAAD;
        for byte in &mut payload {
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            *byte = x as u8;
        }

        info!("defmt stress: seq={=u32} p={=[u8]}", seq, &payload[..]);

        seq = seq.wrapping_add(1);
        Timer::after_millis(STRESS_PERIOD_MS).await;
    }
}
