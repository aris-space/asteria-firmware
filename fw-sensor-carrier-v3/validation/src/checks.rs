use defmt::{error, info};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::mode::Async;
use embassy_stm32::sdmmc::Error as SdError;
use embassy_stm32::sdmmc::sd::{Addressable, CmdBlock, DataBlock, StorageDevice};
use embassy_stm32::time::{Hertz, mhz};
use embassy_stm32::usart::UartRx;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Timer, with_timeout};
use lsm6dso32::spi::Lsm6Dso32SpiInterface;
use lsm6dso32::{Lsm6dso32, Uninitialised};
use lsm303agr::Lsm303agr;
use ms5607::Ms5607;
use sht4x::Sht4xAsync;
use ublox::{FixedLinearBuffer, Parser};

use crate::resources::buses::{SharedI2c, SharedI2cBus};
use crate::resources::buzzer::BuzzerPwm;
use crate::resources::flash::{BoardFlash, WINBOND_MANUFACTURER_ID};
use crate::resources::sd::Sd;
use crate::resources::sensors::SpiDevice;

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

/// Cycle each LED on its own and beep the buzzer. These have no electrical
/// readback, so the operator confirms them visually/audibly.
pub async fn leds_and_buzzer(
    green: &mut Output<'static>,
    yellow: &mut Output<'static>,
    red: &mut Output<'static>,
    buzzer: &mut BuzzerPwm,
) {
    info!("leds/buzzer: watch the board and listen for the beep");

    // All three together, then each individually.
    green.set_high();
    yellow.set_high();
    red.set_high();
    Timer::after(Duration::from_millis(800)).await;
    green.set_low();
    yellow.set_low();
    red.set_low();
    Timer::after(Duration::from_millis(300)).await;

    for (name, led) in [
        ("green", &mut *green),
        ("yellow", &mut *yellow),
        ("red", &mut *red),
    ] {
        info!("leds: {} on", name);
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(200)).await;
    }

    // Passive piezo: sweep a couple of tones so it audibly buzzes.
    info!("buzzer: beep");
    for freq in [2400u32, 3200] {
        beep(buzzer, freq, 200).await;
        Timer::after(Duration::from_millis(80)).await;
    }
}

/// Read the LSM6DSO32 WHO_AM_I register over SPI and check it.
pub async fn imu(spi: SpiDevice, label: &str) -> bool {
    let iface = Lsm6Dso32SpiInterface { spi };
    let mut sensor = Lsm6dso32::<_, Uninitialised>::new(iface);

    match sensor.inner_mut().who_am_i().read_async().await {
        Ok(reg) if reg.ident() == LSM6DSO32_WHO_AM_I => {
            info!("{}: OK (WHO_AM_I 0x{:02x})", label, reg.ident());
            true
        }
        Ok(reg) => {
            error!("{}: wrong WHO_AM_I 0x{:02x}", label, reg.ident());
            false
        }
        Err(e) => {
            error!(
                "{}: WHO_AM_I read failed: {}",
                label,
                defmt::Debug2Format(&e)
            );
            false
        }
    }
}

/// Init the MS5607, which reads its factory PROM (it has no WHO_AM_I register).
pub async fn barometer(bus: SharedI2cBus, label: &str) -> bool {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    let sensor = Ms5607::new(i2c, false);

    match sensor.init(&mut Delay).await {
        Ok(_) => {
            info!("{}: OK (PROM read)", label);
            true
        }
        Err(e) => {
            error!("{}: PROM read failed: {}", label, defmt::Debug2Format(&e));
            false
        }
    }
}

/// Read the LSM303AGR magnetometer WHO_AM_I register and check it.
pub async fn magnetometer(bus: SharedI2cBus, label: &str) -> bool {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    let mut sensor = Lsm303agr::new_with_i2c(i2c);

    match sensor.magnetometer_id().await {
        Ok(id) if id.is_correct() => {
            info!("{}: OK (WHO_AM_I 0x{:02x})", label, id.raw());
            true
        }
        Ok(id) => {
            error!("{}: wrong WHO_AM_I 0x{:02x}", label, id.raw());
            false
        }
        Err(e) => {
            error!(
                "{}: WHO_AM_I read failed: {}",
                label,
                defmt::Debug2Format(&e)
            );
            false
        }
    }
}

/// Read the SHT4x serial number to confirm it responds on the bus.
pub async fn sht4x(bus: SharedI2cBus, label: &str) -> bool {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    let mut sensor = Sht4xAsync::new(i2c);

    match sensor.serial_number(&mut Delay).await {
        Ok(serial) => {
            info!("{}: OK (serial {})", label, serial);
            true
        }
        Err(e) => {
            error!("{}: serial read failed: {}", label, defmt::Debug2Format(&e));
            false
        }
    }
}

/// Read bytes from a GNSS receiver and try to parse at least one u-blox packet.
/// Receiving any data confirms the UART link; parsed packets are a bonus.
pub async fn gnss(mut rx: UartRx<'static, Async>, label: &str) -> bool {
    let mut buf = [0u8; 256];
    let mut parse_buf = [0u8; 1024];
    let mut parser = Parser::new(FixedLinearBuffer::new(&mut parse_buf));

    let mut total_bytes = 0usize;
    let mut packets = 0usize;

    // Up to ~2.4 s of listening, bailing early once we have a parsed packet.
    for _ in 0..8 {
        match with_timeout(Duration::from_millis(300), rx.read_until_idle(&mut buf)).await {
            Ok(Ok(n)) if n > 0 => {
                total_bytes += n;
                let mut consumed = parser.consume(&buf[..n]);
                while let Some(packet) = consumed.next() {
                    if packet.is_ok() {
                        packets += 1;
                    }
                }
            }
            Ok(Err(e)) => error!("{}: UART read error: {}", label, defmt::Debug2Format(&e)),
            _ => {}
        }
        if packets > 0 {
            break;
        }
    }

    if total_bytes > 0 {
        info!(
            "{}: OK ({} bytes, {} ublox packets)",
            label, total_bytes, packets
        );
        true
    } else {
        error!("{}: no data received (UART / baudrate / wiring?)", label);
        false
    }
}

/// Read the flash JEDEC manufacturer byte over OCTOSPI and confirm it is Winbond.
pub fn flash(mut flash: BoardFlash) -> bool {
    let manufacturer = flash.manufacturer_id();
    info!("flash: JEDEC manufacturer 0x{:02x}", manufacturer);

    if manufacturer == WINBOND_MANUFACTURER_ID {
        info!("flash: OK (Winbond)");
        true
    } else {
        // 0x00 = line stayed low (chip not driving); 0xff = idle high / floating.
        error!(
            "flash: not responding (mfr 0x{:02x}, expected 0xEF Winbond)",
            manufacturer
        );
        false
    }
}

/// Issue one SDMMC command at the register level with bounded polling, so it can
/// never hang. Returns the short response register on success, `None` on timeout.
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
            // R3 (ACMD41) carries no CRC, so CCRCFAIL is expected — accept it.
            if s.cmdrend() || s.ccrcfail() {
                return Some(SDMMC1.respr(0).read().cardstatus());
            }
        } else if s.cmdsent() {
            return Some(0);
        }
    }
    None
}

/// Validate the SD card at the register level (embassy's H723 SDMMC driver hangs,
/// but the peripheral works): CMD0 -> CMD8 (a v2 card responds) -> ACMD41 (the
/// card finishes power-up). Fully bounded, so it cannot hang.
pub async fn sd_card(_sdmmc: Sd, detect: Input<'static>, _power: Output<'static>) -> bool {
    info!(
        "sd: card-detect reads {}",
        if detect.is_high() { "high" } else { "low" }
    );

    // SD_VDD (PD6) was enabled in setup(); let it settle before talking.
    Timer::after(Duration::from_millis(250)).await;

    // Add the MCU's internal pull-ups on CMD/DAT. With the board's 10k externals
    // plus the EMIF filter (U14) capacitance, the lines may not pull high fast
    // enough when released — which would make the controller read an all-low CMD
    // line as a zero response (CMDREND + all-zero data, exactly what we see).
    {
        use embassy_stm32::pac::gpio::vals::Pupdr;
        use embassy_stm32::pac::{GPIOC, GPIOD};
        for pin in [8usize, 9, 10, 11] {
            GPIOC.pupdr().modify(|w| w.set_pupdr(pin, Pupdr::PULL_UP)); // DAT0-3 = PC8-11
        }
        GPIOD.pupdr().modify(|w| w.set_pupdr(2, Pupdr::PULL_UP)); // CMD = PD2
    }

    // Read the real pin levels now: pull-ups on, SDMMC not yet driving. A healthy
    // line idles HIGH (1). Any line reading 0 here is physically held low (short /
    // dead pull-up / bad U14) — a definitive board fault, no scope needed.
    {
        let pc = embassy_stm32::pac::GPIOC.idr().read().0;
        let pd = embassy_stm32::pac::GPIOD.idr().read().0;
        info!(
            "sd: idle CMD={} CLK={} DAT0={} D1={} D2={} D3={} | PWR_EN(PD6)={} DET(PD3)={}",
            (pd >> 2) & 1,
            (pc >> 12) & 1,
            (pc >> 8) & 1,
            (pc >> 9) & 1,
            (pc >> 10) & 1,
            (pc >> 11) & 1,
            (pd >> 6) & 1, // TPS22918 enable — should be 1 (high = card powered)
            (pd >> 3) & 1, // card detect
        );
    }

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

    // CMD8: voltage 2.7-3.6 V + check pattern 0xAA. A valid echo = v2 card present
    // and the link works.
    let cmd8 = sd_cmd(8, 0x1AA, true).unwrap_or(0);
    let cmd8_sta = SDMMC1.star().read().0;
    info!(
        "sd: CMD8 resp r0={:#x} r1={:#x} r2={:#x} r3={:#x} STAR={:#x}",
        SDMMC1.respr(0).read().0,
        SDMMC1.respr(1).read().0,
        SDMMC1.respr(2).read().0,
        SDMMC1.respr(3).read().0,
        cmd8_sta
    );
    if cmd8_sta & 0x40 == 0 {
        // No CMDREND -> the card never sent a response.
        error!(
            "sd: CMD8 no response (STAR={:#x}) — no card seated?",
            cmd8_sta
        );
        return false;
    }
    if cmd8 & 0xFF == 0xAA {
        info!("sd: CMD8 OK (SDv2, echo {:#x})", cmd8);
    } else {
        info!(
            "sd: CMD8 responded (echo {:#x} read quirk); continuing",
            cmd8
        );
    }

    // ACMD41 (CMD55 then ACMD41) until the card reports power-up complete.
    let mut ready = false;
    for _ in 0..500 {
        sd_cmd(55, 0, true); // CMD55, RCA = 0
        // HCS + full 2.7-3.6 V window; OCR bit 31 = power-up done.
        match sd_cmd(41, 0x40FF_8000, true) {
            Some(ocr) if ocr & 0x8000_0000 != 0 => {
                ready = true;
                info!("sd: ACMD41 ready (OCR {:#x})", ocr);
                break;
            }
            Some(_) => {}
            None => break,
        }
        Timer::after(Duration::from_millis(4)).await;
    }

    if ready {
        info!("sd: OK (card present and initialized)");
        true
    } else {
        error!("sd: ACMD41 never completed power-up");
        false
    }
}

/// Acquire the card, round-trip the last block, restore it, and return
/// `(capacity_MiB, data_matched)`.
async fn sd_roundtrip(sdmmc: &mut Sd, cmd_block: &mut CmdBlock) -> Result<(u64, bool), SdError> {
    let mut storage = StorageDevice::new_sd_card(sdmmc, cmd_block, mhz(25)).await?;

    let size = storage.card().size();
    // Use the last block to stay clear of any partition table / filesystem.
    let lba = ((size / 512).saturating_sub(1)) as u32;

    let mut original = DataBlock::new();
    storage.read_block(lba, &mut original).await?;

    let mut pattern = DataBlock::new();
    for (i, word) in pattern.0.iter_mut().enumerate() {
        *word = 0xA5A5_0000 ^ i as u32;
    }

    storage.write_block(lba, &pattern).await?;
    let mut readback = DataBlock::new();
    storage.read_block(lba, &mut readback).await?;

    // Restore the original contents before reporting the comparison.
    storage.write_block(lba, &original).await?;

    Ok((size / (1024 * 1024), readback.0 == pattern.0))
}

/// Announce the verdict: solid green after a rising chime on success, solid red
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
    } else {
        for freq in [400u32, 300] {
            beep(buzzer, freq, 250).await;
            Timer::after(Duration::from_millis(60)).await;
        }
        red.set_high();
    }
}
