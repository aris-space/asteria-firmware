#![no_std]
#![no_main]

use embassy_executor::Spawner;

use defmt_rtt as _;
use embassy_time::Timer;
use panic_probe as _;

#[macro_use]
mod fmt;

mod assign_resources;
#[cfg(feature = "use-i2c4")]
mod bounce_i2c;
mod checks;
mod clocks;
mod resources;
mod support;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;

use crate::resources::usb::UsbDriver;

/// Mirror the defmt/RTT log over USB-C (CDC-ACM serial). Built by hand, not via
/// `run!`, for a named port; spawned early so its pipe buffers lines until connected.
#[embassy_executor::task]
async fn usb_logger_task(driver: UsbDriver) {
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Asteria");
    config.product = Some("Sensor Board Validation");
    config.serial_number = Some("sensor-board");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 0]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([]),
        CONTROL_BUF.init([0; 64]),
    );
    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);
    let mut device = builder.build();

    let logger = embassy_usb_logger::with_class!(8192, log::LevelFilter::Info, class);
    embassy_futures::join::join(device.run(), logger).await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_stm32::init(clocks::clocks_config());

    let r = resources::split(p);

    spawner.spawn(usb_logger_task(r.usb.setup()).expect("spawn usb logger task"));

    // wait for the usb logger task to start up properly
    Timer::after_secs(1).await;

    println!("=== sensor-board hardware validation ===");

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

    let (sdmmc, sd_detect, sd_power) = r.sd_card.setup();
    all_passed &= checks::sd_card(sdmmc, sd_detect, sd_power).await;

    let verdict = if all_passed {
        "=== CORE CHECKS PASSED ==="
    } else {
        "=== SOME CHECKS FAILED (scroll up for details) ==="
    };
    println!("{}", verdict);

    checks::announce(&mut green, &mut yellow, &mut red, &mut buzzer, all_passed).await;

    loop {
        core::future::pending::<()>().await;
    }
}
