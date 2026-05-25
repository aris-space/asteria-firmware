#![no_std]
#![no_main]

use defmt::println;
use embassy_executor::Spawner;

use defmt_rtt as _;
use panic_probe as _;

mod checks;
mod resources;
mod support;

mod clocks {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../shared/stm32h723_clocks.rs"
    ));
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let mut clock_config = clocks::clocks_config();
    {
        // SDMMC's default kernel clock is PLL1_Q, which this board runs at 240 MHz
        // (shared with SPI). That exceeds the SDMMC kernel-clock max (~200 MHz), so
        // the peripheral misbehaves and init hangs. Give SDMMC a dedicated PLL2_R
        // at 200 MHz instead. (The firmware will need the same once it uses the SD
        // card.)
        use embassy_stm32::rcc::{Pll, PllDiv, PllMul, PllPreDiv, PllSource, mux};
        clock_config.rcc.pll2 = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL25, // 16 MHz * 25 = 400 MHz VCO
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV2),
            divr: Some(PllDiv::DIV2), // 400 / 2 = 200 MHz -> SDMMC kernel
        });
        clock_config.rcc.mux.sdmmcsel = mux::Sdmmcsel::PLL2_R;
    }
    let p = embassy_stm32::init(clock_config);

    let r = resources::split(p);

    println!("=== sensor-carrier-v3 hardware validation ===");

    // Visual/audible peripherals first; the operator confirms these by eye/ear.
    let mut green = r.green_led.setup();
    let mut yellow = r.yellow_led.setup();
    let mut red = r.red_led.setup();
    let mut buzzer = r.buzzer.setup();
    checks::leds_and_buzzer(&mut green, &mut yellow, &mut red, &mut buzzer).await;

    let mut all_passed = true;

    // SPI IMUs: SPI link + WHO_AM_I, a DRDY rising edge on INT1, then a live
    // accel/gyro/temperature read.
    let (imu1_spi, imu1_int1) = r.imu1.setup();
    all_passed &= checks::imu(imu1_spi, imu1_int1, "imu1").await;
    let (imu2_spi, imu2_int1) = r.imu2.setup();
    all_passed &= checks::imu(imu2_spi, imu2_int1, "imu2").await;

    // I2C buses: a barometer + magnetometer on each.
    let bus1 = r.bus1.setup();
    let bus2 = r.bus2.setup();
    all_passed &= checks::barometer(bus1, "barometer0 (bus1)").await;
    all_passed &= checks::barometer(bus2, "barometer1 (bus2)").await;
    all_passed &= checks::magnetometer(bus1, "magnetometer0 (bus1)").await;
    all_passed &= checks::magnetometer(bus2, "magnetometer1 (bus2)").await;
    all_passed &= checks::sht4x(bus1, "sht4x0 (bus1)").await;
    all_passed &= checks::sht4x(bus2, "sht4x1 (bus2)").await;

    // UART GNSS receivers.
    let (_gps1_tx, gps1_rx) = r.gps1_uart.setup().split();
    all_passed &= checks::gnss(gps1_rx, "gnss1").await;
    let (_gps2_tx, gps2_rx) = r.gps2_uart.setup().split();
    all_passed &= checks::gnss(gps2_rx, "gnss2").await;

    // OCTOSPI flash.
    all_passed &= checks::flash(r.flash.setup());

    let verdict = if all_passed {
        "=== CORE CHECKS PASSED ==="
    } else {
        "=== SOME CHECKS FAILED (scroll up for details) ==="
    };
    println!("{}", verdict);

    // Announce the verdict for the core peripherals BEFORE touching the SD card.
    // embassy's blocking SD init can spin forever on a card that won't finish its
    // ACMD41 power-up, and that must not swallow the LED/buzzer verdict.
    checks::announce(&mut green, &mut yellow, &mut red, &mut buzzer, all_passed).await;

    // SD card runs last, as an addendum. If it completes and fails, downgrade the
    // verdict LED to red; the register-level check is bounded so it cannot hang.
    let (sdmmc, sd_detect, sd_power) = r.sd_card.setup();
    if !checks::sd_card(sdmmc, sd_detect, sd_power).await && all_passed {
        green.set_low();
        red.set_high();
    }

    loop {
        core::future::pending::<()>().await;
    }
}
