//! On-board W25Q01JV NOR flash on OCTOSPI1, over quad SPI. IO2 (/WP) and IO3
//! (/HOLD) double as data lines; external pull-ups (R49/R50) keep them high during
//! the single-line phases until Quad Enable is set. The OCTOSPI bring-up (quad
//! init, QE, JEDEC probe) matches the validation crate; the addressed transfers
//! use 4-byte-address opcodes because the part is 128 MiB and 24-bit addressing
//! reaches only its bottom 16 MiB. Reads use Fast Read Quad Output (0x6C); program
//! and erase stay single-line. The driver implements `embedded-storage-async` so
//! `sequential-storage` can sit on top unchanged.

use embassy_stm32::mode::Blocking;
use embassy_stm32::ospi::{
    AddressSize, Config as OspiConfig, DummyCycles, MemorySize, Ospi, OspiError, OspiWidth,
    TransferConfig,
};
use embassy_stm32::peripherals::OCTOSPI1;
use embassy_time::Timer;
use embedded_storage_async::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use super::Flash;

pub const PAGE_SIZE: u32 = 256;
pub const SECTOR_SIZE: u32 = 4096;
pub const CAPACITY: u32 = 128 * 1024 * 1024;

/// JEDEC manufacturer byte a healthy W25Q01JV returns: Winbond.
pub const WINBOND_MANUFACTURER_ID: u8 = 0xEF;

/// JEDEC memory-type byte for the W25Q *IM (DTR) variants; the non-DTR IQ parts
/// report 0x40 instead.
pub const W25Q_IM_MEMORY_TYPE: u8 = 0x70;

/// JEDEC capacity byte for the 1Gbit W25Q01JV.
pub const W25Q01JV_CAPACITY: u8 = 0x21;

mod cmd {
    pub const READ_JEDEC_ID: u8 = 0x9F;
    pub const WRITE_ENABLE: u8 = 0x06;
    pub const READ_STATUS_1: u8 = 0x05;
    pub const READ_STATUS_2: u8 = 0x35;
    pub const WRITE_STATUS_2: u8 = 0x31;
    // 4-byte-address opcodes: the chip is 128 MiB, so 24-bit (16 MiB) addressing
    // can't reach most of it. These take a 32-bit address regardless of the chip's
    // 3-/4-byte mode bit, so no mode switch is needed.
    pub const FAST_READ_QUAD_OUTPUT_4B: u8 = 0x6C;
    pub const PAGE_PROGRAM_4B: u8 = 0x12;
    pub const SECTOR_ERASE_4K_4B: u8 = 0x21;
}

mod status {
    /// Quad Enable (S9), in status register 2.
    pub const QE: u8 = 0x02;
    /// Write In Progress (S0), in status register 1.
    pub const WIP: u8 = 0x01;
}

/// JEDEC ID bytes read from the flash.
pub struct JedecId {
    pub manufacturer: u8,
    pub memory_type: u8,
    pub capacity: u8,
}

pub struct BoardFlash {
    ospi: Ospi<'static, OCTOSPI1, Blocking>,
}

#[derive(Debug, defmt::Format)]
pub enum FlashError {
    Ospi(OspiError),
    OutOfBounds,
    NotAligned,
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            FlashError::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            FlashError::NotAligned => NorFlashErrorKind::NotAligned,
            FlashError::Ospi(_) => NorFlashErrorKind::Other,
        }
    }
}

/// Single-line instruction only, no address or data (e.g. write-enable).
fn cmd_only(instruction: u8) -> TransferConfig {
    TransferConfig {
        iwidth: OspiWidth::SING,
        instruction: Some(instruction as u32),
        ..Default::default()
    }
}

/// Single-line instruction then single-line data, no address (status and JEDEC
/// reads, status-register writes).
fn cmd_data(instruction: u8) -> TransferConfig {
    TransferConfig {
        dwidth: OspiWidth::SING,
        ..cmd_only(instruction)
    }
}

/// Single-line instruction with a 4-byte address, optionally carrying single-line
/// data (page program, sector erase).
fn cmd_addr(instruction: u8, address: u32, with_data: bool) -> TransferConfig {
    TransferConfig {
        adwidth: OspiWidth::SING,
        address: Some(address),
        adsize: AddressSize::_32bit,
        dwidth: if with_data {
            OspiWidth::SING
        } else {
            OspiWidth::NONE
        },
        ..cmd_only(instruction)
    }
}

/// Fast Read Quad Output, 4-byte address: instruction and address single-line,
/// data over all four IO lines after 8 dummy cycles.
fn quad_read(address: u32) -> TransferConfig {
    TransferConfig {
        adwidth: OspiWidth::SING,
        address: Some(address),
        adsize: AddressSize::_32bit,
        dwidth: OspiWidth::QUAD,
        dummy: DummyCycles::_8,
        ..cmd_only(cmd::FAST_READ_QUAD_OUTPUT_4B)
    }
}

impl BoardFlash {
    /// Read the 3-byte JEDEC ID (0x9F): manufacturer, memory type, capacity.
    pub fn read_jedec_id(&mut self) -> JedecId {
        let mut buf = [0u8; 3];
        let _ = self
            .ospi
            .blocking_read(&mut buf, cmd_data(cmd::READ_JEDEC_ID));
        JedecId {
            manufacturer: buf[0],
            memory_type: buf[1],
            capacity: buf[2],
        }
    }

    /// Read status register 1 (bit0 = WIP, bit1 = WEL).
    pub fn status(&mut self) -> u8 {
        self.read_status(cmd::READ_STATUS_1)
    }

    /// Set the Quad Enable bit so IO2/IO3 act as data lines. Idempotent: skips the
    /// (non-volatile) status-register write when QE is already set. Returns whether
    /// QE reads back set.
    pub fn enable_quad(&mut self) -> bool {
        if self.read_status(cmd::READ_STATUS_2) & status::QE != 0 {
            return true;
        }
        let _ = self.write_enable();
        let _ = self
            .ospi
            .blocking_write(&[status::QE], cmd_data(cmd::WRITE_STATUS_2));
        // Wait for the non-volatile status write to finish (WIP clears). Each poll is
        // a SPI read so it self-paces; the bound is a timeout against a wedged bit.
        const WIP_POLL_LIMIT: u32 = 100_000;
        for _ in 0..WIP_POLL_LIMIT {
            if self.read_status(cmd::READ_STATUS_1) & status::WIP == 0 {
                return self.read_status(cmd::READ_STATUS_2) & status::QE != 0;
            }
        }
        false
    }

    fn write_enable(&mut self) -> Result<(), FlashError> {
        self.ospi
            .blocking_command(&cmd_only(cmd::WRITE_ENABLE))
            .map_err(FlashError::Ospi)
    }

    /// Poll the WIP bit until the chip finishes its current program/erase,
    /// yielding between polls so other tasks keep running.
    async fn wait_idle(&mut self) {
        while self.status() & status::WIP != 0 {
            Timer::after_micros(200).await;
        }
    }

    fn read_status(&mut self, instruction: u8) -> u8 {
        let mut b = [0u8; 1];
        let _ = self.ospi.blocking_read(&mut b, cmd_data(instruction));
        b[0]
    }
}

impl ErrorType for BoardFlash {
    type Error = FlashError;
}

impl ReadNorFlash for BoardFlash {
    const READ_SIZE: usize = 1;

    async fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        if bytes.is_empty() {
            return Ok(());
        }
        if offset + bytes.len() as u32 > CAPACITY {
            return Err(FlashError::OutOfBounds);
        }
        // OCTOSPI on H7 has no embassy DMA path (it routes through MDMA, which
        // embassy doesn't implement), so the transfer is polled; yield afterwards so
        // a flash scan's back-to-back reads don't starve the executor.
        let result = self
            .ospi
            .blocking_read(bytes, quad_read(offset))
            .map_err(FlashError::Ospi);
        embassy_futures::yield_now().await;
        result
    }

    fn capacity(&self) -> usize {
        CAPACITY as usize
    }
}

impl NorFlash for BoardFlash {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = SECTOR_SIZE as usize;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if !from.is_multiple_of(SECTOR_SIZE) || !to.is_multiple_of(SECTOR_SIZE) {
            return Err(FlashError::NotAligned);
        }
        if to > CAPACITY || from > to {
            return Err(FlashError::OutOfBounds);
        }
        let mut addr = from;
        while addr < to {
            self.write_enable()?;
            self.ospi
                .blocking_command(&cmd_addr(cmd::SECTOR_ERASE_4K_4B, addr, false))
                .map_err(FlashError::Ospi)?;
            self.wait_idle().await;
            addr += SECTOR_SIZE;
        }
        Ok(())
    }

    async fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if bytes.is_empty() {
            return Ok(());
        }
        if offset + bytes.len() as u32 > CAPACITY {
            return Err(FlashError::OutOfBounds);
        }
        let mut addr = offset;
        let mut rest = bytes;
        while !rest.is_empty() {
            // A page program cannot cross a 256-byte page boundary; clamp to it.
            let page_left = PAGE_SIZE - (addr % PAGE_SIZE);
            let n = core::cmp::min(page_left as usize, rest.len());
            let (chunk, tail) = rest.split_at(n);
            self.write_enable()?;
            self.ospi
                .blocking_write(chunk, cmd_addr(cmd::PAGE_PROGRAM_4B, addr, true))
                .map_err(FlashError::Ospi)?;
            self.wait_idle().await;
            addr += n as u32;
            rest = tail;
        }
        Ok(())
    }
}

impl MultiwriteNorFlash for BoardFlash {}

fn config() -> OspiConfig {
    OspiConfig {
        device_size: MemorySize::_128MiB,
        // PRESCALER divides the kernel clock by value+1, so 7 = /8:
        // hclk3 (240 MHz) / 8 = 30 MHz, conservative for bring-up.
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

        let mut flash = BoardFlash { ospi };

        let id = flash.read_jedec_id();
        if id.manufacturer == WINBOND_MANUFACTURER_ID
            && id.memory_type == W25Q_IM_MEMORY_TYPE
            && id.capacity == W25Q01JV_CAPACITY
        {
            defmt::info!(
                "flash: Winbond W25Q01JV-IM, 1Gbit DTR (JEDEC {=u8:#04x} {=u8:#04x} {=u8:#04x})",
                id.manufacturer,
                id.memory_type,
                id.capacity
            );
        } else if id.manufacturer == WINBOND_MANUFACTURER_ID {
            defmt::error!(
                "flash: Winbond present, but unexpected JEDEC id ({=u8:#04x} {=u8:#04x} {=u8:#04x})",
                id.manufacturer,
                id.memory_type,
                id.capacity
            );
        } else {
            defmt::error!(
                "flash: not responding (JEDEC {=u8:#04x} {=u8:#04x} {=u8:#04x})",
                id.manufacturer,
                id.memory_type,
                id.capacity
            );
        }

        // IO2/IO3 only carry data once Quad Enable is set, so do it before the
        // first quad read.
        if !flash.enable_quad() {
            defmt::error!("flash: could not set quad-enable (QE) bit");
        }

        flash
    }
}
