use block_device_adapters::BufStream;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::mode::Async;
use embassy_stm32::sdmmc::sd::{Addressable, CmdBlock, StorageDevice};
use embassy_stm32::time::{Hertz, mhz};
use embassy_stm32::usart::UartRx;
use embassy_time::{Delay, Duration, Timer, with_timeout};
use embedded_fatfs::{FileSystem, FsOptions};
use embedded_hal_async::i2c::I2c;
use embedded_io_async::{Read, Seek, SeekFrom, Write};
use embedded_partitions::mbr::Mbr;
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
use crate::resources::flash::{
    BoardFlash, W25Q_IM_MEMORY_TYPE, W25Q01JV_CAPACITY, WINBOND_MANUFACTURER_ID,
};
use crate::resources::sd::Sd;
use crate::resources::sensors::SpiDevice;
use crate::support::{SAMPLES, median};

/// Sound the passive piezo at `freq` for `ms` milliseconds.
async fn beep(buzzer: &mut BuzzerPwm, freq: u32, ms: u64) {
    buzzer.set_frequency(Hertz(freq));
    let half = buzzer.ch4().max_duty_cycle() / 2;
    buzzer.ch4().set_duty_cycle(half);
    buzzer.ch4().enable();
    Timer::after(Duration::from_millis(ms)).await;
    buzzer.ch4().disable();
}

/// Light all LEDs and beep. No electrical readback, so confirmed by eye/ear. The
/// LEDs stay on until `announce` sets the verdict.
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

    // Sweep tones so the passive piezo audibly buzzes.
    for freq in [2400u32, 3200] {
        beep(buzzer, freq, 200).await;
        Timer::after(Duration::from_millis(80)).await;
    }
}

/// Confirm the CPU_FREQ_BOOST option byte is set. It is read-only from firmware
/// (programmed via the debug probe) and is what lets the H723 core run above
/// 520 MHz at VOS0, which the 544 MHz clock config here depends on.
pub fn cpu_freq_boost() -> bool {
    if embassy_stm32::pac::SYSCFG.ur18().read().cpu_freq_boost() {
        info!("cpu_freq_boost: option byte set");
        true
    } else {
        error!("cpu_freq_boost: option byte NOT set (core capped at 520 MHz)");
        false
    }
}

/// Validate an LSM6DSO32: WHO_AM_I over SPI, then configure it, confirm the
/// data-ready interrupt toggles INT1, and read one accel/gyro/temperature sample.
pub async fn imu(spi: SpiDevice, mut int1: ExtiInput<'static, Async>, label: &str) -> bool {
    let iface = Lsm6Dso32SpiInterface { spi };
    let mut sensor = Lsm6dso32::<_, Uninitialised>::new(iface);

    /// Expected LSM6DSO32 WHO_AM_I value.
    const LSM6DSO32_WHO_AM_I: u8 = 0x6C;

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
                crate::fmt::Debug2Format(&e)
            );
            return false;
        }
    }

    let mut sensor = match sensor.init(&mut Delay).await {
        Ok(s) => s,
        Err(e) => {
            error!("{}: init failed: {}", label, crate::fmt::Debug2Format(&e));
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
            crate::fmt::Debug2Format(&e)
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
        error!(
            "{}: gyro config failed: {}",
            label,
            crate::fmt::Debug2Format(&e)
        );
        return false;
    }

    // Route accel DRDY to INT1 to exercise the interrupt line, not just SPI.
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
        error!(
            "{}: INT1 config failed: {}",
            label,
            crate::fmt::Debug2Format(&e)
        );
        return false;
    }

    // DRDY is level-held: read once to clear it (INT1 low), then catch the next
    // rising edge.
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

    // Per-sample magnitudes/temperature, then the median of each.
    let mut accel = [0.0f32; SAMPLES];
    let mut gyro = [0.0f32; SAMPLES];
    let mut temps = [0.0f32; SAMPLES];
    for i in 0..SAMPLES {
        let acc = match sensor.read_acceleration().await {
            Ok(a) => a,
            Err(e) => {
                error!(
                    "{}: accel read failed: {}",
                    label,
                    crate::fmt::Debug2Format(&e)
                );
                return false;
            }
        };
        let g = match sensor.read_angular_rate().await {
            Ok(g) => g,
            Err(e) => {
                error!(
                    "{}: gyro read failed: {}",
                    label,
                    crate::fmt::Debug2Format(&e)
                );
                return false;
            }
        };
        let t = match sensor.read_temperature().await {
            Ok(t) => t,
            Err(e) => {
                error!(
                    "{}: temp read failed: {}",
                    label,
                    crate::fmt::Debug2Format(&e)
                );
                return false;
            }
        };
        accel[i] = libm::sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
        gyro[i] = libm::sqrtf(g.x * g.x + g.y * g.y + g.z * g.z);
        temps[i] = t.value;
        // ~one ODR period, so each read is fresh.
        Timer::after(Duration::from_millis(10)).await;
    }
    info!(
        "{}: |accel| = {} g, |gyro| = {} dps, temp = {} C",
        label,
        median(&mut accel),
        median(&mut gyro),
        median(&mut temps)
    );

    int1_ok
}

/// Init the MS5607 (no WHO_AM_I, so init reads and verifies its factory PROM
/// instead), then take one pressure/temperature measurement.
pub async fn barometer<I: I2c>(i2c: I, label: &str) -> bool {
    let sensor = Ms5607::new(i2c, false);

    let mut sensor = match sensor.init(&mut Delay).await {
        Ok(s) => s,
        Err(e) => {
            error!(
                "{}: PROM read failed: {}",
                label,
                crate::fmt::Debug2Format(&e)
            );
            return false;
        }
    };

    let mut pressure = [0.0f32; SAMPLES];
    let mut temps = [0.0f32; SAMPLES];
    for i in 0..SAMPLES {
        match sensor.measure(Oversampling::Osr2048, &mut Delay).await {
            Ok(m) => {
                pressure[i] = m.pressure_mbar;
                temps[i] = m.temperature_c;
            }
            Err(e) => {
                error!(
                    "{}: measure failed: {}",
                    label,
                    crate::fmt::Debug2Format(&e)
                );
                return false;
            }
        }
    }
    info!(
        "{}: pressure = {} mbar, temp = {} C",
        label,
        median(&mut pressure),
        median(&mut temps)
    );
    true
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
                crate::fmt::Debug2Format(&e)
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

    // Per-sample |B|, then the median to reject interference spikes.
    let mut mags = [0.0f32; SAMPLES];
    for slot in mags.iter_mut() {
        // > one 100 Hz period, so each read is fresh.
        Timer::after(Duration::from_millis(15)).await;
        match sensor.magnetic_field().await {
            Ok(f) => {
                let (x, y, z) = (f.x_nt() as f32, f.y_nt() as f32, f.z_nt() as f32);
                *slot = libm::sqrtf(x * x + y * y + z * z);
            }
            Err(e) => {
                error!("{}: read failed: {}", label, crate::fmt::Debug2Format(&e));
                return false;
            }
        }
    }
    info!("{}: |B| = {} nT", label, median(&mut mags));
    true
}

/// Read the SHT4x serial number to confirm it responds, then take one
/// temperature/humidity measurement.
pub async fn sht4x<I: I2c>(i2c: I, label: &str) -> bool {
    let mut sensor = Sht4xAsync::new(i2c);

    match sensor.serial_number(&mut Delay).await {
        Ok(serial) => info!("{}: serial {}", label, serial),
        Err(e) => {
            error!(
                "{}: serial read failed: {}",
                label,
                crate::fmt::Debug2Format(&e)
            );
            return false;
        }
    }

    let mut temps = [0.0f32; SAMPLES];
    let mut humidity = [0.0f32; SAMPLES];
    for i in 0..SAMPLES {
        match sensor.measure(Precision::High, &mut Delay).await {
            Ok(m) => {
                temps[i] = m.temperature_celsius().to_num::<f32>();
                humidity[i] = m.humidity_percent().to_num::<f32>();
            }
            Err(e) => {
                error!(
                    "{}: measure failed: {}",
                    label,
                    crate::fmt::Debug2Format(&e)
                );
                return false;
            }
        }
    }
    info!(
        "{}: temp = {} C, RH = {} %",
        label,
        median(&mut temps),
        median(&mut humidity)
    );
    true
}

/// Validate a GNSS receiver: UART link up, valid UBX packets, and UBX-NAV-STATUS
/// present. A receiver that talks but never sends NAV-STATUS is misconfigured.
pub async fn gnss(mut rx: UartRx<'static, Async>, label: &str) -> bool {
    let mut buf = [0u8; 256];
    let mut parse_buf = [0u8; 1024];
    let mut parser = Parser::new(FixedLinearBuffer::new(&mut parse_buf));

    let mut total_bytes = 0usize;
    let mut packets = 0usize;
    let mut nav_status = false;

    // NAV-STATUS is ~1 Hz; listen a few seconds, bail once we decode one.
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
            Ok(Err(e)) => error!(
                "{}: UART read error: {}",
                label,
                crate::fmt::Debug2Format(&e)
            ),
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

/// Confirm the W25Q01JV over OCTOSPI: read its 3-byte JEDEC ID, then enable quad
/// mode and read a span over all four IO lines, comparing it against a single-line
/// read of the same span to exercise IO2/IO3.
pub fn flash(mut flash: BoardFlash) -> bool {
    let id = flash.read_jedec_id();
    if id.manufacturer != WINBOND_MANUFACTURER_ID
        || id.memory_type != W25Q_IM_MEMORY_TYPE
        || id.capacity != W25Q01JV_CAPACITY
    {
        // 0x00 = line stayed low (chip not driving); 0xff = idle high / floating.
        error!(
            "flash: unexpected JEDEC ID (0x{:02x} 0x{:02x} 0x{:02x}, expected 0xEF 0x70 0x21)",
            id.manufacturer, id.memory_type, id.capacity
        );
        return false;
    }
    info!(
        "flash: Winbond W25Q01JV-IM, 1Gbit DTR (JEDEC 0x{:02x} 0x{:02x} 0x{:02x})",
        id.manufacturer, id.memory_type, id.capacity
    );

    if !flash.enable_quad() {
        error!("flash: could not set quad-enable (QE) bit");
        return false;
    }

    // A single-line and a quad read of the same span must agree; even on erased
    // flash (all 0xff) this catches an IO2/IO3 line stuck low, shorted, or swapped.
    let mut single = [0u8; 32];
    let mut quad = [0u8; 32];
    flash.read_data(0x00_0000, &mut single);
    flash.quad_read(0x00_0000, &mut quad);
    if single == quad {
        info!("flash: quad read matches single-line read (IO2/IO3 OK)");
        true
    } else {
        error!("flash: quad vs single-line readback mismatch (IO2/IO3 wiring?)");
        false
    }
}

/// Round-trip HELLO_WORLD.txt (write then read back) on the first FAT partition to
/// prove the card is writable. Card must be FAT-formatted; the MBR is parsed since
/// embassy is raw-LBA.
///
/// TODO: assert card-detect (PD3) and SD_VDD (PD6), once their polarities are known,
/// to tell an absent/unpowered card from a failed init.
pub async fn sd_card(mut sdmmc: Sd, detect: Input<'static>, _power: Output<'static>) -> bool {
    info!(
        "sd: detect = {}",
        if detect.is_high() { "high" } else { "low" }
    );

    // SD_VDD (PD6) is on from setup(); let the rail settle.
    Timer::after(Duration::from_millis(250)).await;

    let mut cmd_block = CmdBlock::new();
    let card = match with_timeout(
        Duration::from_secs(2),
        StorageDevice::new_sd_card(&mut sdmmc, &mut cmd_block, mhz(25)),
    )
    .await
    {
        Ok(Ok(card)) => card,
        Ok(Err(e)) => {
            error!("sd: init failed: {}", crate::fmt::Debug2Format(&e));
            return false;
        }
        Err(_) => {
            error!("sd: init timed out (card seated / powered?)");
            return false;
        }
    };
    info!("sd: init OK ({} MiB)", card.card().size() / (1024 * 1024));

    let mbr = match Mbr::new(BufStream::<_, 512>::new(card)).await {
        Ok(m) => m,
        Err(e) => {
            error!("sd: read MBR failed: {}", crate::fmt::Debug2Format(&e));
            return false;
        }
    };
    let Some(idx) = mbr.iter_used().find(|(_, p)| p.is_fat()).map(|(i, _)| i) else {
        error!("sd: no FAT partition found (is the card FAT-formatted?)");
        return false;
    };
    let slice = match mbr.into_partition(idx).await {
        Ok(s) => s,
        Err(e) => {
            error!(
                "sd: open partition failed: {}",
                crate::fmt::Debug2Format(&e)
            );
            return false;
        }
    };
    let fs = match FileSystem::new(slice, FsOptions::new()).await {
        Ok(fs) => fs,
        Err(e) => {
            error!("sd: mount FAT failed: {}", crate::fmt::Debug2Format(&e));
            return false;
        }
    };

    const MSG: &[u8] = b"hello world\n";

    let root = fs.root_dir();
    let mut file = match root.create_file("HELLO_WORLD.txt").await {
        Ok(f) => f,
        Err(e) => {
            error!(
                "sd: create HELLO_WORLD.txt failed: {}",
                crate::fmt::Debug2Format(&e)
            );
            return false;
        }
    };
    if let Err(e) = file.truncate().await {
        error!("sd: truncate failed: {}", crate::fmt::Debug2Format(&e));
        return false;
    }
    if let Err(e) = file.write_all(MSG).await {
        error!("sd: write failed: {}", crate::fmt::Debug2Format(&e));
        return false;
    }
    if let Err(e) = file.flush().await {
        error!("sd: flush failed: {}", crate::fmt::Debug2Format(&e));
        return false;
    }

    // Read it back and confirm.
    if let Err(e) = file.seek(SeekFrom::Start(0)).await {
        error!("sd: seek failed: {}", crate::fmt::Debug2Format(&e));
        return false;
    }
    let mut buf = [0u8; MSG.len()];
    if let Err(e) = file.read_exact(&mut buf).await {
        error!("sd: read-back failed: {}", crate::fmt::Debug2Format(&e));
        return false;
    }
    if buf != *MSG {
        error!("sd: read-back mismatch");
        return false;
    }

    // Drop file/dir (they borrow fs) before unmounting.
    drop(file);
    drop(root);
    if let Err(e) = fs.unmount().await {
        error!("sd: unmount/flush failed: {}", crate::fmt::Debug2Format(&e));
        return false;
    }

    info!("sd: OK (wrote and read back HELLO_WORLD.txt)");
    true
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
