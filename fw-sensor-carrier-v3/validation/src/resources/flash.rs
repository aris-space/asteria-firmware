//! On-board W25Q01JV NOR flash on OCTOSPI1, over quad SPI. IO2 (/WP) and IO3
//! (/HOLD) double as data lines; external pull-ups (R49/R50) keep them high during
//! the single-line phases until Quad Enable is set. Mirrors the firmware bring-up.

use embassy_stm32::mode::Blocking;
use embassy_stm32::ospi::{
    AddressSize, Config as OspiConfig, DummyCycles, MemorySize, Ospi, OspiWidth, TransferConfig,
};
use embassy_stm32::peripherals::OCTOSPI1;

use super::Flash;

/// JEDEC manufacturer byte a healthy W25Q01JV returns: Winbond.
pub const WINBOND_MANUFACTURER_ID: u8 = 0xEF;

/// JEDEC memory-type byte for the W25Q *IM (DTR) variants; the non-DTR IQ parts
/// report 0x40 instead.
pub const W25Q_IM_MEMORY_TYPE: u8 = 0x70;

/// JEDEC capacity byte for the 1Gbit W25Q01JV.
pub const W25Q01JV_CAPACITY: u8 = 0x21;

const READ_JEDEC_ID: u8 = 0x9F;
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

/// JEDEC ID bytes read from the flash.
pub struct JedecId {
    pub manufacturer: u8,
    pub memory_type: u8,
    pub capacity: u8,
}

pub struct BoardFlash {
    ospi: Ospi<'static, OCTOSPI1, Blocking>,
}

impl BoardFlash {
    /// Read the 3-byte JEDEC ID (0x9F): manufacturer, memory type, capacity.
    pub fn read_jedec_id(&mut self) -> JedecId {
        let mut buf = [0u8; 3];
        let _ = self.ospi.blocking_read(
            &mut buf,
            TransferConfig {
                iwidth: OspiWidth::SING,
                instruction: Some(READ_JEDEC_ID as u32),
                dwidth: OspiWidth::SING,
                ..Default::default()
            },
        );
        JedecId {
            manufacturer: buf[0],
            memory_type: buf[1],
            capacity: buf[2],
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
        // Wait for the non-volatile status write to finish (WIP clears). Each poll is
        // a SPI read so it self-paces; the bound is a timeout against a wedged bit.
        const WIP_POLL_LIMIT: u32 = 100_000;
        for _ in 0..WIP_POLL_LIMIT {
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
        // PRESCALER divides the kernel clock by value+1, so 7 = /8:
        // hclk3 (272 MHz) / 8 = 34 MHz, conservative for bring-up.
        clock_prescaler: 7,
        ..Default::default()
    }
}

impl Flash {
    pub fn setup(self) -> BoardFlash {
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
