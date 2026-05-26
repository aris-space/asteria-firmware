//! On-board W25Q01JV NOR flash on OCTOSPI1, brought up over quad SPI. IO2 (/WP)
//! and IO3 (/HOLD) are OCTOSPI data lines; external 10k pull-ups (R49/R50) hold
//! them high through the single-line phases (ID read, QE-enable) before the Quad
//! Enable bit is set. Mirrors the firmware's flash bring-up.

use embassy_stm32::mode::Blocking;
use embassy_stm32::ospi::{
    AddressSize, Config as OspiConfig, DummyCycles, MemorySize, Ospi, OspiWidth, TransferConfig,
};
use embassy_stm32::peripherals::OCTOSPI1;

use super::Flash;

/// JEDEC manufacturer byte a healthy W25Q01JV returns: Winbond.
pub const WINBOND_MANUFACTURER_ID: u8 = 0xEF;

/// Device ID this board's W25Q01JV returns to the 0x90 command.
pub const W25Q01JV_DEVICE_ID: u8 = 0x20;

const READ_MANUFACTURER_DEVICE_ID: u8 = 0x90;
const WRITE_ENABLE: u8 = 0x06;
const READ_STATUS_1: u8 = 0x05;
const READ_STATUS_2: u8 = 0x35;
const WRITE_STATUS_2: u8 = 0x31;
const READ_DATA: u8 = 0x03;
const FAST_READ_QUAD_OUTPUT: u8 = 0x6B;

/// Quad Enable (S9), in status register 2.
const STATUS_2_QE: u8 = 0x02;
/// Write In Progress (S0), in status register 1.
const STATUS_1_WIP: u8 = 0x01;

/// Manufacturer and device bytes read from the flash.
pub struct FlashId {
    pub manufacturer: u8,
    pub device: u8,
}

pub struct BoardFlash {
    ospi: Ospi<'static, OCTOSPI1, Blocking>,
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

    /// Set the Quad Enable bit so IO2/IO3 act as data lines. Idempotent: skips the
    /// (non-volatile) status-register write when QE is already set. Returns whether
    /// QE reads back set.
    pub fn enable_quad(&mut self) -> bool {
        if self.read_status(READ_STATUS_2) & STATUS_2_QE != 0 {
            return true;
        }
        self.send_command(WRITE_ENABLE);
        let _ = self.ospi.blocking_write(
            &[STATUS_2_QE],
            TransferConfig {
                iwidth: OspiWidth::SING,
                instruction: Some(WRITE_STATUS_2 as u32),
                dwidth: OspiWidth::SING,
                ..Default::default()
            },
        );
        // The status-register write is non-volatile; spin until WIP clears (each
        // status read is itself a SPI transaction, so this paces the poll).
        for _ in 0..100_000 {
            if self.read_status(READ_STATUS_1) & STATUS_1_WIP == 0 {
                return self.read_status(READ_STATUS_2) & STATUS_2_QE != 0;
            }
        }
        false
    }

    /// Read over a single data line (0x03), for comparison against [`quad_read`].
    pub fn read_data(&mut self, address: u32, buf: &mut [u8]) {
        let _ = self.ospi.blocking_read(
            buf,
            TransferConfig {
                iwidth: OspiWidth::SING,
                instruction: Some(READ_DATA as u32),
                adwidth: OspiWidth::SING,
                address: Some(address),
                adsize: AddressSize::_24bit,
                dwidth: OspiWidth::SING,
                ..Default::default()
            },
        );
    }

    /// Fast Read Quad Output (0x6B): command and address single-line, data over all
    /// four IO lines. The only read here that exercises IO2/IO3.
    pub fn quad_read(&mut self, address: u32, buf: &mut [u8]) {
        let _ = self.ospi.blocking_read(
            buf,
            TransferConfig {
                iwidth: OspiWidth::SING,
                instruction: Some(FAST_READ_QUAD_OUTPUT as u32),
                adwidth: OspiWidth::SING,
                address: Some(address),
                adsize: AddressSize::_24bit,
                dwidth: OspiWidth::QUAD,
                dummy: DummyCycles::_8,
                ..Default::default()
            },
        );
    }

    fn send_command(&mut self, instruction: u8) {
        let _ = self.ospi.blocking_command(&TransferConfig {
            iwidth: OspiWidth::SING,
            instruction: Some(instruction as u32),
            ..Default::default()
        });
    }

    fn read_status(&mut self, instruction: u8) -> u8 {
        let mut b = [0u8; 1];
        let _ = self.ospi.blocking_read(
            &mut b,
            TransferConfig {
                iwidth: OspiWidth::SING,
                instruction: Some(instruction as u32),
                dwidth: OspiWidth::SING,
                ..Default::default()
            },
        );
        b[0]
    }
}

fn config() -> OspiConfig {
    OspiConfig {
        device_size: MemorySize::_128MiB,
        // hclk3 (272 MHz) / 8 = 34 MHz: conservative for bring-up.
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
        // IO2 (/WP) and IO3 (/HOLD) become OCTOSPI data lines; external pull-ups
        // hold them high during the single-line phases before QE is set.
        let ospi = Ospi::new_blocking_quadspi(
            self.periph,
            self.sck,
            self.io0,
            self.io1,
            self.wp,
            self.hold,
            self.ncs,
            config(),
        );

        BoardFlash { ospi }
    }
}
