// TODO: verify on hw with the Split 4 V3

use bitflags::bitflags;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart};
use embassy_time::{Duration, Instant, Timer, with_timeout};

use crate::resources::camera::CameraUart;

const BOOT_TIMEOUT: Duration = Duration::from_secs(15);
const BOOT_POLL_INTERVAL: Duration = Duration::from_millis(100);
const FILE_CLOSE_DELAY: Duration = Duration::from_secs(2);

const HEADER: u8 = 0xCC;
const GET_DEVICE_INFO: u8 = 0x00;
const CAMERA_CONTROL: u8 = 0x01;
const SIMULATE_POWER_BUTTON: u8 = 0x01;
const START_RECORDING: u8 = 0x03;
const STOP_RECORDING: u8 = 0x04;

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    pub struct Features: u16 {
        const SIMULATE_POWER_BUTTON = 1 << 0;
        const START_RECORDING = 1 << 6;
        const STOP_RECORDING = 1 << 7;
    }
}

/// GET_DEVICE_INFO reply: header, protocol version, 16-bit feature mask, CRC.
const DEVICE_INFO_LEN: usize = 5;
const REPLY_TIMEOUT: Duration = Duration::from_millis(500);

#[derive(Debug, defmt::Format)]
pub enum Error {
    Uart(usart::Error),
    NoReply,
    InvalidReply,
    Unsupported,
}

impl From<usart::Error> for Error {
    fn from(e: usart::Error) -> Self {
        Self::Uart(e)
    }
}

pub struct RunCam {
    power: Output<'static>,
    uart: CameraUart,
}

impl RunCam {
    pub fn new(power: Output<'static>, uart: CameraUart) -> Self {
        Self { power, uart }
    }

    /// Powers the camera and opens a fresh UART to it.
    pub fn power_on(&mut self) -> PoweredRunCam<'_> {
        self.power.set_high();
        PoweredRunCam {
            power: &mut self.power,
            uart: self.uart.open(),
        }
    }
}

pub struct PoweredRunCam<'d> {
    power: &'d mut Output<'static>,
    uart: Uart<'d, Async>,
}

impl PoweredRunCam<'_> {
    /// Closes the UART first so TX doesn't back-power the camera, then cuts power.
    pub fn power_off(self) {
        let Self { power, uart } = self;
        drop(uart);
        power.set_low();
    }

    /// Wait until the camera has booted and fetch the device info.
    pub async fn wait_until_booted(&mut self) -> Result<Features, Error> {
        let deadline = Instant::now() + BOOT_TIMEOUT;
        loop {
            match self.request_device_info().await {
                Ok(features) => return Ok(features),
                Err(e) if Instant::now() >= deadline => return Err(e),
                Err(_) => Timer::after(BOOT_POLL_INTERVAL).await,
            }
        }
    }

    /// Starts the recording (if supported) and returns immediately.
    pub async fn start_recording(&mut self) -> Result<(), Error> {
        self.send(&[CAMERA_CONTROL, START_RECORDING]).await
    }

    /// Stops the recording of the camera (if supported), and waits for the file to be closed.
    pub async fn stop_recording(&mut self, features: Features) -> Result<(), Error> {
        if features.contains(Features::STOP_RECORDING) {
            self.send(&[CAMERA_CONTROL, STOP_RECORDING]).await?;
        } else if features.contains(Features::START_RECORDING | Features::SIMULATE_POWER_BUTTON) {
            // the power button toggles, so only press it if we could start recording
            self.send(&[CAMERA_CONTROL, SIMULATE_POWER_BUTTON]).await?;
        } else {
            return Err(Error::Unsupported);
        }
        Timer::after(FILE_CLOSE_DELAY).await;
        Ok(())
    }

    async fn request_device_info(&mut self) -> Result<Features, Error> {
        self.send(&[GET_DEVICE_INFO]).await?;

        let mut buf = [0u8; 32];
        let len = with_timeout(REPLY_TIMEOUT, self.uart.read_until_idle(&mut buf))
            .await
            .map_err(|_| Error::NoReply)??;

        for frame in buf[..len].windows(DEVICE_INFO_LEN) {
            if frame[0] == HEADER && crc8_dvb_s2(&frame[..4]) == frame[4] {
                let features = u16::from_le_bytes([frame[2], frame[3]]);
                return Ok(Features::from_bits_retain(features));
            }
        }
        Err(Error::InvalidReply)
    }

    async fn send(&mut self, command: &[u8]) -> Result<(), Error> {
        let mut frame = [0u8; 8];
        frame[0] = HEADER;
        frame[1..=command.len()].copy_from_slice(command);
        let crc_index = command.len() + 1;
        frame[crc_index] = crc8_dvb_s2(&frame[..crc_index]);

        self.uart.write(&frame[..=crc_index]).await?;
        Ok(())
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
