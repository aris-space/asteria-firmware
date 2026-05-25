//! On-board W25Q256JV NOR flash on OCTOSPI1, operated in plain single-line SPI
//! (1-1-1) mode. IO2 (/WP) and IO3 (/HOLD) are held high as GPIOs so the part
//! behaves like a classic SPI flash. Mirrors the firmware's flash bring-up; the
//! validation only needs the JEDEC presence probe.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::ospi::{
    AddressSize, Config as OspiConfig, MemorySize, Ospi, OspiWidth, TransferConfig,
};
use embassy_stm32::peripherals::OCTOSPI1;

use super::Flash;

/// JEDEC manufacturer byte a healthy W25Q256JV returns: Winbond.
pub const WINBOND_MANUFACTURER_ID: u8 = 0xEF;

/// Device ID this board's W25Q256JV returns to the 0x90 command.
pub const W25Q256JV_DEVICE_ID: u8 = 0x20;

const READ_MANUFACTURER_DEVICE_ID: u8 = 0x90;

/// Manufacturer and device bytes read from the flash.
pub struct FlashId {
    pub manufacturer: u8,
    pub device: u8,
}

pub struct BoardFlash {
    ospi: Ospi<'static, OCTOSPI1, Blocking>,
    _hold: Output<'static>,
    _wp: Output<'static>,
}

impl BoardFlash {
    /// Read manufacturer + device ID via 0x90. Its 24-bit address phase warms up
    /// read sampling so both bytes latch; the address-less 0x9F read garbles
    /// everything past byte 0.
    pub fn read_id(&mut self) -> FlashId {
        let mut buf = [0u8; 2];
        let _ = self.ospi.blocking_read(
            &mut buf,
            TransferConfig {
                iwidth: OspiWidth::SING,
                instruction: Some(READ_MANUFACTURER_DEVICE_ID as u32),
                adwidth: OspiWidth::SING,
                address: Some(0x00_0000), // returns manufacturer then device
                adsize: AddressSize::_24bit,
                dwidth: OspiWidth::SING,
                ..Default::default()
            },
        );
        FlashId {
            manufacturer: buf[0],
            device: buf[1],
        }
    }
}

fn config() -> OspiConfig {
    OspiConfig {
        device_size: MemorySize::_32MiB,
        // hclk3 (240 MHz) / 8 = 30 MHz: conservative for bring-up.
        clock_prescaler: 7,
        // Sample MISO half a clock late so the flash output (delayed by the round
        // trip through the series resistors) is settled when latched; without it
        // reads past the first byte garble.
        sample_shifting: true,
        ..Default::default()
    }
}

impl Flash {
    pub fn setup(self) -> BoardFlash {
        // /WP and /HOLD are unused in 1-line SPI; hold high so the chip doesn't
        // write-protect or pause.
        let wp = Output::new(self.wp, Level::High, Speed::VeryHigh);
        let hold = Output::new(self.hold, Level::High, Speed::VeryHigh);

        let ospi = Ospi::new_blocking_singlespi(
            self.periph,
            self.sck,
            self.io0,
            self.io1,
            self.ncs,
            config(),
        );

        BoardFlash {
            ospi,
            _hold: hold,
            _wp: wp,
        }
    }
}
