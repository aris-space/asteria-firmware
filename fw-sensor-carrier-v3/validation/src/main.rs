#![no_std]
#![no_main]

use defmt::println;
use embassy_executor::Spawner;

use defmt_rtt as _;
use panic_probe as _;

mod assign_resources;
#[cfg(feature = "use-i2c4")]
mod bounce_i2c;
mod checks;
mod clocks;
mod resources;
mod support;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let p = embassy_stm32::init(clocks::clocks_config());

    let r = resources::split(p);

    println!("=== sensor-carrier-v3 hardware validation ===");

    // Visual/audible first; the operator confirms these by eye/ear.
    let mut green = r.green_led.setup();
    let mut yellow = r.yellow_led.setup();
    let mut red = r.red_led.setup();
    let mut buzzer = r.buzzer.setup();
    checks::leds_and_buzzer(&mut green, &mut yellow, &mut red, &mut buzzer).await;

    let mut all_passed = true;

    let (imu1_spi, imu1_int1) = r.imu1.setup();
    all_passed &= checks::imu(imu1_spi, imu1_int1, "imu1").await;
    let (imu2_spi, imu2_int1) = r.imu2.setup();
    all_passed &= checks::imu(imu2_spi, imu2_int1, "imu2").await;

    // bus1 = I2C5; bus2 = I2C4 (BDMA, via SRAM4 bounce) or bridged I2C2.
    let bus1 = r.bus1.setup();
    let bus2 = r.bus2.setup();
    all_passed &= checks::barometer(I2cDevice::new(bus1), "barometer0 (bus1)").await;
    all_passed &= checks::barometer(I2cDevice::new(bus2), "barometer1 (bus2)").await;
    all_passed &= checks::magnetometer(I2cDevice::new(bus1), "magnetometer0 (bus1)").await;
    all_passed &= checks::magnetometer(I2cDevice::new(bus2), "magnetometer1 (bus2)").await;
    all_passed &= checks::sht4x(I2cDevice::new(bus1), "sht4x0 (bus1)").await;
    all_passed &= checks::sht4x(I2cDevice::new(bus2), "sht4x1 (bus2)").await;

    let (_gps1_tx, gps1_rx) = r.gps1_uart.setup().split();
    all_passed &= checks::gnss(gps1_rx, "gnss1").await;
    let (_gps2_tx, gps2_rx) = r.gps2_uart.setup().split();
    all_passed &= checks::gnss(gps2_rx, "gnss2").await;

    all_passed &= checks::flash(r.flash.setup());

    let verdict = if all_passed {
        "=== CORE CHECKS PASSED ==="
    } else {
        "=== SOME CHECKS FAILED (scroll up for details) ==="
    };
    println!("{}", verdict);

    // Announce the core verdict before the SD card. The idiomatic SDMMC init
    // busy-waits on the command path (a timeout can't cancel a synchronous poll),
    // so a missing or bad card can stall here and must not swallow the LED/buzzer
    // result.
    checks::announce(&mut green, &mut yellow, &mut red, &mut buzzer, all_passed).await;

    // SD runs last; on a clean failure flip the board to red. A wholly
    // unresponsive card may stall this step, but the core verdict is already out.
    let (sdmmc, sd_detect, sd_power) = r.sd_card.setup();
    if !checks::sd_card(sdmmc, sd_detect, sd_power).await && all_passed {
        green.set_low();
        yellow.set_low();
        red.set_high();
    }

    loop {
        core::future::pending::<()>().await;
    }
}
