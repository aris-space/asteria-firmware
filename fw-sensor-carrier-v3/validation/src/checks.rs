use defmt::{error, info};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::mode::Async;
use embassy_stm32::time::Hertz;
use embassy_stm32::usart::UartRx;
use embassy_time::{Delay, Duration, Timer, with_timeout};
use embedded_hal_async::i2c::I2c;
use lsm6dso32::spi::Lsm6Dso32SpiInterface;
use lsm6dso32::{
    AccelerometerFullScale, AccelerometerOdr, GyroscopeFullScale, GyroscopeOdr, Int1Config,
    Lsm6dso32, Uninitialised,
};
use lsm303agr::{Lsm303agr, MagMode, MagOutputDataRate};
use ms5607::{Ms5607, Oversampling};
use sht4x::{Precision, Sht4xAsync};
use ublox::{FixedLinearBuffer, PacketRef, Parser};

use crate::resources::buzzer::BuzzerPwm;
use crate::resources::flash::{BoardFlash, W25Q256JV_DEVICE_ID, WINBOND_MANUFACTURER_ID};
use crate::resources::sd::Sd;
use crate::resources::sensors::SpiDevice;
use crate::support::{
    ACCEL_MAGNITUDE_G, BARO_PRESSURE_MBAR, BARO_TEMP_C, GYRO_MAGNITUDE_DPS, IMU_TEMP_C,
    MAG_MAGNITUDE_NT, MAG_SAMPLES, MAG_TRIM, SAMPLES, SHT_RH_PCT, SHT_TEMP_C, trimmed_mean,
};

/// Expected LSM6DSO32 WHO_AM_I value.
const LSM6DSO32_WHO_AM_I: u8 = 0x6C;

/// Sound the passive piezo at `freq` for `ms` milliseconds.
async fn beep(buzzer: &mut BuzzerPwm, freq: u32, ms: u64) {
    buzzer.set_frequency(Hertz(freq));
    let half = buzzer.ch4().max_duty_cycle() / 2;
    buzzer.ch4().set_duty_cycle(half);
    buzzer.ch4().enable();
    Timer::after(Duration::from_millis(ms)).await;
    buzzer.ch4().disable();
}

/// Light all three LEDs and beep the buzzer. These have no electrical readback,
/// so the operator confirms them visually/audibly. The LEDs stay on for the rest
/// of the run; `announce` later collapses them to the pass/fail verdict.
pub async fn leds_and_buzzer(
    green: &mut Output<'static>,
    yellow: &mut Output<'static>,
    red: &mut Output<'static>,
    buzzer: &mut BuzzerPwm,
) {
    info!("leds/buzzer: all LEDs on, listen for the beep");

    green.set_high();
    yellow.set_high();
    red.set_high();

    // Passive piezo: sweep a couple of tones so it audibly buzzes.
    for freq in [2400u32, 3200] {
        beep(buzzer, freq, 200).await;
        Timer::after(Duration::from_millis(80)).await;
    }
}

/// Validate an LSM6DSO32: WHO_AM_I over SPI, then configure it, confirm the
/// data-ready interrupt toggles INT1, and read one accel/gyro/temperature sample.
pub async fn imu(spi: SpiDevice, mut int1: ExtiInput<'static, Async>, label: &str) -> bool {
    let iface = Lsm6Dso32SpiInterface { spi };
    let mut sensor = Lsm6dso32::<_, Uninitialised>::new(iface);

    match sensor.inner_mut().who_am_i().read_async().await {
        Ok(reg) if reg.ident() == LSM6DSO32_WHO_AM_I => {
            info!("{}: WHO_AM_I 0x{:02x}", label, reg.ident());
        }
        Ok(reg) => {
            error!("{}: wrong WHO_AM_I 0x{:02x}", label, reg.ident());
            return false;
        }
        Err(e) => {
            error!(
                "{}: WHO_AM_I read failed: {}",
                label,
                defmt::Debug2Format(&e)
            );
            return false;
        }
    }

    let mut sensor = match sensor.init(&mut Delay).await {
        Ok(s) => s,
        Err(e) => {
            error!("{}: init failed: {}", label, defmt::Debug2Format(&e));
            return false;
        }
    };

    if let Err(e) = sensor
        .set_accelerometer_odr_and_full_scale(
            Some(AccelerometerOdr::Hz104),
            Some(AccelerometerFullScale::G8),
        )
        .await
    {
        error!(
            "{}: accel config failed: {}",
            label,
            defmt::Debug2Format(&e)
        );
        return false;
    }
    if let Err(e) = sensor
        .set_gyroscope_odr_and_full_scale(
            Some(GyroscopeOdr::Hz104),
            Some(GyroscopeFullScale::Dps2000),
        )
        .await
    {
        error!("{}: gyro config failed: {}", label, defmt::Debug2Format(&e));
        return false;
    }

    // Route the accelerometer data-ready flag to INT1 so we exercise the physical
    // interrupt line, not just the SPI link.
    if let Err(e) = sensor
        .configure_interrupts(
            Some(Int1Config {
                drdy_xl: true,
                ..Default::default()
            }),
            None,
        )
        .await
    {
        error!("{}: INT1 config failed: {}", label, defmt::Debug2Format(&e));
        return false;
    }

    // DRDY is level-held until the sample is read, so clear it (INT1 drops low),
    // then wait for the next conversion to drive a fresh rising edge. At 104 Hz a
    // new sample arrives every ~10 ms, well inside the timeout.
    let _ = sensor.read_acceleration().await;
    let int1_ok = with_timeout(Duration::from_millis(150), int1.wait_for_rising_edge())
        .await
        .is_ok();
    if int1_ok {
        info!("{}: INT1 data-ready edge seen", label);
    } else {
        error!(
            "{}: no INT1 edge within 150 ms (DRDY routing / INT1 wiring?)",
            label
        );
    }

    // Average several samples for a steadier estimate, then range-check.
    let (mut ax, mut ay, mut az) = (0.0f32, 0.0f32, 0.0f32);
    let (mut gx, mut gy, mut gz) = (0.0f32, 0.0f32, 0.0f32);
    let mut temp_c = 0.0f32;
    for _ in 0..SAMPLES {
        let acc = match sensor.read_acceleration().await {
            Ok(a) => a,
            Err(e) => {
                error!("{}: accel read failed: {}", label, defmt::Debug2Format(&e));
                return false;
            }
        };
        let gyro = match sensor.read_angular_rate().await {
            Ok(g) => g,
            Err(e) => {
                error!("{}: gyro read failed: {}", label, defmt::Debug2Format(&e));
                return false;
            }
        };
        let temp = match sensor.read_temperature().await {
            Ok(t) => t,
            Err(e) => {
                error!("{}: temp read failed: {}", label, defmt::Debug2Format(&e));
                return false;
            }
        };
        ax += acc.x;
        ay += acc.y;
        az += acc.z;
        gx += gyro.x;
        gy += gyro.y;
        gz += gyro.z;
        temp_c += temp.value;
        // One ODR period (104 Hz) so each read sees a fresh sample.
        Timer::after(Duration::from_millis(10)).await;
    }
    let n = SAMPLES as f32;
    let accel_mag = libm::sqrtf((ax * ax + ay * ay + az * az) / (n * n));
    let gyro_mag = libm::sqrtf((gx * gx + gy * gy + gz * gz) / (n * n));
    let temp_c = temp_c / n;
    info!(
        "{}: |accel| = {} g, |gyro| = {} dps, temp = {} C",
        label, accel_mag, gyro_mag, temp_c
    );

    let ranges_ok = ACCEL_MAGNITUDE_G.check(label, "|accel|", accel_mag)
        & GYRO_MAGNITUDE_DPS.check(label, "|gyro|", gyro_mag)
        & IMU_TEMP_C.check(label, "temp", temp_c);

    int1_ok && ranges_ok
}

/// Init the MS5607 (it has no WHO_AM_I, so init reads/verifies its factory PROM),
/// then take one pressure/temperature measurement.
pub async fn barometer<I: I2c>(i2c: I, label: &str) -> bool {
    let sensor = Ms5607::new(i2c, false);

    let mut sensor = match sensor.init(&mut Delay).await {
        Ok(s) => s,
        Err(e) => {
            error!("{}: PROM read failed: {}", label, defmt::Debug2Format(&e));
            return false;
        }
    };

    let mut pressure = 0.0f32;
    let mut temp = 0.0f32;
    for _ in 0..SAMPLES {
        match sensor.measure(Oversampling::Osr2048, &mut Delay).await {
            Ok(m) => {
                pressure += m.pressure_mbar;
                temp += m.temperature_c;
            }
            Err(e) => {
                error!("{}: measure failed: {}", label, defmt::Debug2Format(&e));
                return false;
            }
        }
    }
    let n = SAMPLES as f32;
    let (pressure, temp) = (pressure / n, temp / n);
    info!("{}: pressure = {} mbar, temp = {} C", label, pressure, temp);
    BARO_PRESSURE_MBAR.check(label, "pressure", pressure) & BARO_TEMP_C.check(label, "temp", temp)
}

/// Check the LSM303AGR WHO_AM_I, then put the magnetometer into continuous mode
/// and read one magnetic-field sample.
pub async fn magnetometer<I: I2c>(i2c: I, label: &str) -> bool {
    let mut sensor = Lsm303agr::new_with_i2c(i2c);

    match sensor.magnetometer_id().await {
        Ok(id) if id.is_correct() => info!("{}: WHO_AM_I 0x{:02x}", label, id.raw()),
        Ok(id) => {
            error!("{}: wrong WHO_AM_I 0x{:02x}", label, id.raw());
            return false;
        }
        Err(e) => {
            error!(
                "{}: WHO_AM_I read failed: {}",
                label,
                defmt::Debug2Format(&e)
            );
            return false;
        }
    }

    if sensor.init().await.is_err() {
        error!("{}: init failed", label);
        return false;
    }
    let mut sensor = match sensor.into_mag_continuous().await {
        Ok(s) => s,
        Err(_) => {
            error!("{}: failed to enter continuous mode", label);
            return false;
        }
    };
    if sensor
        .set_mag_mode_and_odr(
            &mut Delay,
            MagMode::HighResolution,
            MagOutputDataRate::Hz100,
        )
        .await
        .is_err()
    {
        error!("{}: mag ODR config failed", label);
        return false;
    }

    // Collect per-sample magnitudes over a longer window, then take a trimmed mean
    // so an occasional interference spike does not skew |B|.
    let mut mags = [0.0f32; MAG_SAMPLES];
    for slot in mags.iter_mut() {
        // Wait past one 100 Hz sample period so each read is a fresh measurement.
        Timer::after(Duration::from_millis(15)).await;
        match sensor.magnetic_field().await {
            Ok(f) => {
                let (x, y, z) = (f.x_nt() as f32, f.y_nt() as f32, f.z_nt() as f32);
                *slot = libm::sqrtf(x * x + y * y + z * z);
            }
            Err(e) => {
                error!("{}: read failed: {}", label, defmt::Debug2Format(&e));
                return false;
            }
        }
    }
    let mag = trimmed_mean(&mut mags, MAG_TRIM);
    info!("{}: |B| = {} nT", label, mag);
    MAG_MAGNITUDE_NT.check(label, "|B|", mag)
}

/// Read the SHT4x serial number to confirm it responds, then take one
/// temperature/humidity measurement.
pub async fn sht4x<I: I2c>(i2c: I, label: &str) -> bool {
    let mut sensor = Sht4xAsync::new(i2c);

    match sensor.serial_number(&mut Delay).await {
        Ok(serial) => info!("{}: serial {}", label, serial),
        Err(e) => {
            error!("{}: serial read failed: {}", label, defmt::Debug2Format(&e));
            return false;
        }
    }

    let mut temp = 0.0f32;
    let mut humidity = 0.0f32;
    for _ in 0..SAMPLES {
        match sensor.measure(Precision::High, &mut Delay).await {
            Ok(m) => {
                temp += m.temperature_celsius().to_num::<f32>();
                humidity += m.humidity_percent().to_num::<f32>();
            }
            Err(e) => {
                error!("{}: measure failed: {}", label, defmt::Debug2Format(&e));
                return false;
            }
        }
    }
    let n = SAMPLES as f32;
    let (temp, humidity) = (temp / n, humidity / n);
    info!("{}: temp = {} C, RH = {} %", label, temp, humidity);
    SHT_TEMP_C.check(label, "temp", temp) & SHT_RH_PCT.check(label, "RH", humidity)
}

/// Validate a GNSS receiver: confirm the UART link, that we decode valid u-blox
/// packets, and specifically that UBX-NAV-STATUS arrives (it carries the fix
/// state we rely on). A receiver that talks but never sends NAV-STATUS is
/// misconfigured.
pub async fn gnss(mut rx: UartRx<'static, Async>, label: &str) -> bool {
    let mut buf = [0u8; 256];
    let mut parse_buf = [0u8; 1024];
    let mut parser = Parser::new(FixedLinearBuffer::new(&mut parse_buf));

    let mut total_bytes = 0usize;
    let mut packets = 0usize;
    let mut nav_status = false;

    // NAV-STATUS is typically emitted at 1 Hz, so listen for a few seconds,
    // bailing as soon as we have decoded one.
    for _ in 0..12 {
        match with_timeout(Duration::from_millis(300), rx.read_until_idle(&mut buf)).await {
            Ok(Ok(n)) if n > 0 => {
                total_bytes += n;
                let mut consumed = parser.consume(&buf[..n]);
                while let Some(packet) = consumed.next() {
                    match packet {
                        Ok(PacketRef::NavStatus(_)) => {
                            nav_status = true;
                            packets += 1;
                        }
                        Ok(_) => packets += 1,
                        Err(_) => {}
                    }
                }
            }
            Ok(Err(e)) => error!("{}: UART read error: {}", label, defmt::Debug2Format(&e)),
            _ => {}
        }
        if nav_status {
            break;
        }
    }

    if total_bytes == 0 {
        error!("{}: no data received (UART / baudrate / wiring?)", label);
        return false;
    }
    if packets == 0 {
        error!(
            "{}: {} bytes but no valid UBX packet decoded (baudrate / protocol?)",
            label, total_bytes
        );
        return false;
    }
    if !nav_status {
        error!(
            "{}: {} UBX packets but no NAV-STATUS: possible GNSS misconfiguration (enable UBX-NAV-STATUS output)",
            label, packets
        );
        return false;
    }

    info!(
        "{}: OK ({} bytes, {} packets, NAV-STATUS received)",
        label, total_bytes, packets
    );
    true
}

/// Read the flash manufacturer + device ID over OCTOSPI and confirm it is the
/// expected Winbond W25Q256JV.
pub fn flash(mut flash: BoardFlash) -> bool {
    let id = flash.read_id();
    info!(
        "flash: manufacturer 0x{:02x}, device 0x{:02x}",
        id.manufacturer, id.device
    );

    if id.manufacturer == WINBOND_MANUFACTURER_ID && id.device == W25Q256JV_DEVICE_ID {
        info!("flash: OK (Winbond W25Q256JV)");
        true
    } else {
        // 0x00 = line stayed low (chip not driving); 0xff = idle high / floating.
        error!(
            "flash: unexpected ID (mfr 0x{:02x} dev 0x{:02x}, expected 0xEF 0x20)",
            id.manufacturer, id.device
        );
        false
    }
}

/// Validate the SD card at the register level (embassy's H723 SDMMC driver hangs,
/// but the peripheral works): CMD0 -> CMD8 (a v2 card responds) -> ACMD41 (the
/// card finishes power-up). Fully bounded, so it cannot hang.
pub async fn sd_card(_sdmmc: Sd, detect: Input<'static>, _power: Output<'static>) -> bool {
    /// Issue one SDMMC command at the register level with bounded polling, so it
    /// can never hang. Returns the short response register on success, `None` on
    /// timeout.
    fn sd_cmd(index: u8, arg: u32, expect_response: bool) -> Option<u32> {
        use embassy_stm32::pac::SDMMC1;

        SDMMC1.icr().write(|w| {
            w.set_cmdsentc(true);
            w.set_cmdrendc(true);
            w.set_ccrcfailc(true);
            w.set_ctimeoutc(true);
        });
        // Wait (bounded) for the command path state machine to be idle.
        for _ in 0..100_000u32 {
            if !SDMMC1.star().read().cpsmact() {
                break;
            }
        }

        SDMMC1.argr().write(|w| w.set_cmdarg(arg));
        SDMMC1.cmdr().write(|w| {
            w.set_cmdindex(index);
            w.set_waitresp(if expect_response { 1 } else { 0 }); // 1 = short response
            w.set_cpsmen(true);
        });

        for _ in 0..5_000_000u32 {
            let s = SDMMC1.star().read();
            if s.ctimeout() {
                return None;
            }
            if expect_response {
                // R3 (ACMD41) carries no CRC, so an expected CCRCFAIL is success.
                if s.cmdrend() || s.ccrcfail() {
                    return Some(SDMMC1.respr(0).read().cardstatus());
                }
            } else if s.cmdsent() {
                return Some(0);
            }
        }
        None
    }

    info!(
        "sd: detect={} pwr_en(PD6)={}",
        if detect.is_high() { "high" } else { "low" },
        (embassy_stm32::pac::GPIOD.idr().read().0 >> 6) & 1
    );

    // SD_VDD (PD6) was enabled in setup(); let it settle before talking.
    Timer::after(Duration::from_millis(250)).await;

    use embassy_stm32::pac::SDMMC1;
    SDMMC1.clkcr().write(|w| {
        w.set_clkdiv(250); // ~init speed; the card responds fine at this rate
        w.set_pwrsav(false);
    });
    SDMMC1.power().modify(|w| w.set_pwrctrl(0b11)); // power on
    Timer::after(Duration::from_millis(10)).await;

    // CMD0: go idle (no response).
    sd_cmd(0, 0, false);
    Timer::after(Duration::from_millis(2)).await;

    // CMD8: voltage 2.7-3.6 V + check pattern 0xAA. A response means a v2 card is
    // present and powered.
    sd_cmd(8, 0x1AA, true);
    if SDMMC1.star().read().0 & 0x40 == 0 {
        error!("sd: no CMD8 response (card unpowered / not seated?)");
        return false;
    }

    // ACMD41 (CMD55 then ACMD41) until the card reports power-up complete.
    for _ in 0..500 {
        sd_cmd(55, 0, true); // CMD55, RCA = 0
        // HCS + full 2.7-3.6 V window; OCR bit 31 = power-up done.
        match sd_cmd(41, 0x40FF_8000, true) {
            Some(ocr) if ocr & 0x8000_0000 != 0 => {
                info!("sd: OK (card powered up)");
                return true;
            }
            Some(_) => {}
            None => break,
        }
        Timer::after(Duration::from_millis(4)).await;
    }

    error!("sd: ACMD41 never completed power-up (SD_VDD not coming up?)");
    false
}

/// Announce the verdict: all LEDs lit after a rising chime on success, solid red
/// after a low warning tone on failure.
pub async fn announce(
    green: &mut Output<'static>,
    yellow: &mut Output<'static>,
    red: &mut Output<'static>,
    buzzer: &mut BuzzerPwm,
    all_passed: bool,
) {
    green.set_low();
    yellow.set_low();
    red.set_low();

    if all_passed {
        for freq in [523u32, 659, 784] {
            beep(buzzer, freq, 140).await;
            Timer::after(Duration::from_millis(40)).await;
        }
        green.set_high();
        yellow.set_high();
        red.set_high();
    } else {
        for freq in [400u32, 300] {
            beep(buzzer, freq, 250).await;
            Timer::after(Duration::from_millis(60)).await;
        }
        red.set_high();
    }
}
