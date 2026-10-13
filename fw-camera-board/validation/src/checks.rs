use defmt::{error, info};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Timer, with_timeout};

use crate::resources::Address;

/// Light the debug LED. No electrical readback, so confirmed by eye. The LED
/// stays on until `announce` sets the verdict.
pub async fn led(led: &mut Output<'static>) {
    info!("led: debug LED on");
    led.set_high();
    Timer::after_millis(500).await;
}

/// Print the jumper address. Any value is valid, so the operator compares it
/// against the bridged jumpers.
pub async fn address(address: Address) {
    let address = address.read().await;
    info!("address: {} (0b{:04b})", address, address);
}

/// Switch the camera on, then ask it for its device info over UART until it
/// answers or the attempts run out.
pub async fn camera(mut power: Output<'static>, mut uart: Uart<'static, Async>) -> bool {
    info!("camera: power on, look for the green camera power LED");
    power.set_high();
    // The camera ignores the UART while it boots.
    Timer::after_secs(3).await;

    for _ in 0..10 {
        if let Some(info) = request_device_info(&mut uart).await {
            info!(
                "camera: RunCam protocol version {}, features 0x{:04x}",
                info.protocol_version, info.features
            );
            return true;
        }
    }

    error!("camera: no device info reply over UART");
    false
}

/// Show the verdict on the debug LED: solid on for a pass, fast blinking for a
/// failure.
pub async fn announce(led: &mut Output<'static>, all_passed: bool) -> ! {
    if all_passed {
        led.set_high();
        loop {
            core::future::pending::<()>().await;
        }
    }

    loop {
        led.toggle();
        Timer::after_millis(100).await;
    }
}

// RunCam Device Protocol, as spoken by the RunCam Split.
const RUNCAM_HEADER: u8 = 0xCC;
const RUNCAM_GET_DEVICE_INFO: u8 = 0x00;

struct DeviceInfo {
    protocol_version: u8,
    features: u16,
}

/// Send one GET_DEVICE_INFO request and wait briefly for the reply.
async fn request_device_info(uart: &mut Uart<'static, Async>) -> Option<DeviceInfo> {
    let command = [RUNCAM_HEADER, RUNCAM_GET_DEVICE_INFO];
    let request = [command[0], command[1], crc8_dvb_s2(&command)];
    if let Err(e) = uart.write(&request).await {
        error!("camera: UART write failed: {}", e);
        return None;
    }

    let mut buf = [0u8; 32];
    let len = match with_timeout(Duration::from_millis(500), uart.read_until_idle(&mut buf)).await {
        Ok(Ok(len)) => len,
        Ok(Err(e)) => {
            error!("camera: UART read failed: {}", e);
            return None;
        }
        Err(_) => return None,
    };

    let reply = find_device_info(&buf[..len]);
    if reply.is_none() {
        error!("camera: no valid device info in {=[u8]:02x}", &buf[..len]);
    }
    reply
}

/// Find a device info reply in `bytes`: header, protocol version, 16-bit
/// feature mask, CRC. Stray bytes around the frame are skipped.
fn find_device_info(bytes: &[u8]) -> Option<DeviceInfo> {
    for frame in bytes.windows(5) {
        if frame[0] == RUNCAM_HEADER && crc8_dvb_s2(&frame[..4]) == frame[4] {
            return Some(DeviceInfo {
                protocol_version: frame[1],
                features: u16::from_le_bytes([frame[2], frame[3]]),
            });
        }
    }
    None
}

/// CRC-8/DVB-S2 (polynomial 0xD5), the checksum of every RunCam frame.
fn crc8_dvb_s2(bytes: &[u8]) -> u8 {
    let mut crc = 0;
    for &byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 {
                (crc << 1) ^ 0xD5
            } else {
                crc << 1
            };
        }
    }
    crc
}
