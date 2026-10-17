// TODO: experimental, not tested on hw yet. start/stop rec over the runcam uart

use defmt::error;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::Uart;
use embassy_time::{Duration, Instant, with_timeout};

pub const BOOT_TIMEOUT: Duration = Duration::from_secs(15);
pub const FILE_CLOSE_DELAY: Duration = Duration::from_secs(2);

const HEADER: u8 = 0xCC;
const GET_DEVICE_INFO: u8 = 0x00;
const CAMERA_CONTROL: u8 = 0x01;
const START_RECORDING: u8 = 0x03;
const STOP_RECORDING: u8 = 0x04;

/// GET_DEVICE_INFO reply: header, protocol version, 16-bit feature mask, CRC.
const DEVICE_INFO_LEN: usize = 5;
const REPLY_TIMEOUT: Duration = Duration::from_millis(500);

pub struct RunCam {
    uart: Uart<'static, Async>,
}

impl RunCam {
    pub fn new(uart: Uart<'static, Async>) -> Self {
        Self { uart }
    }

    pub async fn wait_until_booted(&mut self, timeout: Duration) -> bool {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.request_device_info().await {
                return true;
            }
        }
        false
    }

    pub async fn start_recording(&mut self) {
        self.send(&[CAMERA_CONTROL, START_RECORDING]).await;
    }

    pub async fn stop_recording(&mut self) {
        self.send(&[CAMERA_CONTROL, STOP_RECORDING]).await;
    }

    async fn request_device_info(&mut self) -> bool {
        self.send(&[GET_DEVICE_INFO]).await;

        let mut buf = [0u8; 32];
        let len = match with_timeout(REPLY_TIMEOUT, self.uart.read_until_idle(&mut buf)).await {
            Ok(Ok(len)) => len,
            Ok(Err(e)) => {
                error!("RunCam: UART read failed: {}", e);
                return false;
            }
            Err(_) => return false,
        };

        for frame in buf[..len].windows(DEVICE_INFO_LEN) {
            if frame[0] == HEADER && crc8_dvb_s2(&frame[..4]) == frame[4] {
                return true;
            }
        }
        false
    }

    async fn send(&mut self, command: &[u8]) {
        let mut frame = [0u8; 8];
        frame[0] = HEADER;
        frame[1..=command.len()].copy_from_slice(command);
        let crc_index = command.len() + 1;
        frame[crc_index] = crc8_dvb_s2(&frame[..crc_index]);

        if let Err(e) = self.uart.write(&frame[..=crc_index]).await {
            error!("RunCam: UART write failed: {}", e);
        }
    }
}

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
