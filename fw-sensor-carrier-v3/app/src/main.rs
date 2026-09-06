#![no_std]
#![no_main]

use core::future::pending;

use embassy_executor::Spawner;

mod build_info;
mod built;
mod calibration;
mod filters;
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
    let board = startup::prepare(resources::split(p)).await;
    let level_0_spawner = interrupt_executor!(TIM2, P6);

    startup::spawn_tasks(board, thread_spawner, level_0_spawner).await;

    loop {
        pending::<()>().await;
    }
}
