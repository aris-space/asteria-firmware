//! On-board W25Q256JV NOR flash on OCTOSPI1, operated in plain single-line SPI
//! (1-1-1) mode: only IO0 (DI) and IO1 (DO) carry data. In single-line mode the
//! chip repurposes IO2 as /WP and IO3 as /HOLD, so PA7 and PA6 are held high as
//! GPIOs and the part behaves like a classic SPI flash. The driver implements
//! `embedded-storage-async` so `sequential-storage` can sit on top unchanged.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::ospi::{
    AddressSize, Config as OspiConfig, MemorySize, Ospi, OspiError, OspiWidth, TransferConfig,
};
use embassy_stm32::peripherals::OCTOSPI1;
use embassy_time::Timer;
use embedded_storage_async::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use super::Flash;

pub const PAGE_SIZE: u32 = 256;
pub const SECTOR_SIZE: u32 = 4096;
pub const CAPACITY: u32 = 32 * 1024 * 1024;

/// Mfr + device id a healthy W25Q256JV returns to the 0x90 command: Winbond
/// (0xEF) and device id 0x20. Read over the addressed path, which clocks
/// cleanly. A reading of all-zeros or all-ones means the chip isn't answering
/// (wiring, power, or clock).
pub const EXPECTED_MFR_DEVICE_ID: [u8; 2] = [0xEF, 0x20];

mod cmd {
    pub const WRITE_ENABLE: u8 = 0x06;
    pub const READ_STATUS_1: u8 = 0x05;
    pub const READ_MFR_DEVICE_ID: u8 = 0x90;
    pub const READ_DATA: u8 = 0x03;
    pub const PAGE_PROGRAM: u8 = 0x02;
    pub const SECTOR_ERASE_4K: u8 = 0x20;
}

const STATUS_WIP: u8 = 0x01;

type FlashOspi = Ospi<'static, OCTOSPI1, Blocking>;

pub struct BoardFlash {
    ospi: FlashOspi,
    _hold: Output<'static>,
    _wp: Output<'static>,
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

/// Single-line transfer with an instruction only (e.g. write-enable).
fn cmd_only(instruction: u8) -> TransferConfig {
    TransferConfig {
        iwidth: OspiWidth::SING,
        instruction: Some(instruction as u32),
        ..Default::default()
    }
}

/// Single-line transfer: instruction then read data, no address (status register).
fn cmd_read(instruction: u8) -> TransferConfig {
    TransferConfig {
        iwidth: OspiWidth::SING,
        instruction: Some(instruction as u32),
        dwidth: OspiWidth::SING,
        ..Default::default()
    }
}

/// Single-line transfer with a 24-bit address, optionally carrying data.
fn cmd_addr(instruction: u8, address: u32, with_data: bool) -> TransferConfig {
    TransferConfig {
        iwidth: OspiWidth::SING,
        instruction: Some(instruction as u32),
        adwidth: OspiWidth::SING,
        address: Some(address),
        adsize: AddressSize::_24bit,
        dwidth: if with_data {
            OspiWidth::SING
        } else {
            OspiWidth::NONE
        },
        ..Default::default()
    }
}

impl BoardFlash {
    /// Read Winbond mfr + device id via the 0x90 command. It carries a 24-bit
    /// address, so it rides the addressed read path that clocks cleanly (unlike
    /// the instruction-only 0x9F JEDEC read). Returns `[mfr, device]`;
    /// all-zeros/all-ones means the chip isn't responding.
    pub fn read_mfr_device_id(&mut self) -> [u8; 2] {
        let mut id = [0u8; 2];
        let _ = self
            .ospi
            .blocking_read(&mut id, cmd_addr(cmd::READ_MFR_DEVICE_ID, 0x000000, true));
        id
    }

    /// Read status register 1 (bit0 = WIP, bit1 = WEL).
    pub fn status(&mut self) -> u8 {
        let mut s = [0u8; 1];
        let _ = self
            .ospi
            .blocking_read(&mut s, cmd_read(cmd::READ_STATUS_1));
        s[0]
    }

    fn write_enable(&mut self) -> Result<(), FlashError> {
        self.ospi
            .blocking_command(&cmd_only(cmd::WRITE_ENABLE))
            .map_err(FlashError::Ospi)
    }

    /// Poll the WIP bit until the chip finishes its current program/erase,
    /// yielding between polls so other tasks keep running.
    async fn wait_idle(&mut self) {
        while self.status() & STATUS_WIP != 0 {
            Timer::after_micros(200).await;
        }
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
        // embassy doesn't implement), so the transfer is polled. It's small, but
        // yield afterwards so a flash scan's back-to-back reads don't starve the
        // executor.
        let result = self
            .ospi
            .blocking_read(bytes, cmd_addr(cmd::READ_DATA, offset, true))
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
                .blocking_command(&cmd_addr(cmd::SECTOR_ERASE_4K, addr, false))
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
                .blocking_write(chunk, cmd_addr(cmd::PAGE_PROGRAM, addr, true))
                .map_err(FlashError::Ospi)?;
            self.wait_idle().await;
            addr += n as u32;
            rest = tail;
        }
        Ok(())
    }
}

impl MultiwriteNorFlash for BoardFlash {}

fn ospi_config() -> OspiConfig {
    OspiConfig {
        device_size: MemorySize::_32MiB,
        // hclk3 (240 MHz) / 8 = 30 MHz: conservative for first bring-up.
        clock_prescaler: 7,
        // Sample MISO half a clock late so the flash's output (delayed by the
        // round trip through the 27R series resistors) is settled when latched.
        // Without this, reads past the first byte garble.
        sample_shifting: true,
        ..Default::default()
    }
}

impl Flash {
    pub fn setup(self) -> BoardFlash {
        // IO2 (/WP) and IO3 (/HOLD) are unused in single-line SPI; hold them high
        // so the chip never write-protects or pauses on hold.
        let wp = Output::new(self.wp, Level::High, Speed::VeryHigh);
        let hold = Output::new(self.hold, Level::High, Speed::VeryHigh);

        let ospi = Ospi::new_blocking_singlespi(
            self.periph,
            self.sck,
            self.io0,
            self.io1,
            self.ncs,
            ospi_config(),
        );

        let mut flash = BoardFlash {
            ospi,
            _hold: hold,
            _wp: wp,
        };

        // Probe over the 0x90 (addressed) read: the instruction-only 0x9F JEDEC
        // read mis-clocks its continuation bytes on this OSPI, so those bytes
        // can't be trusted. The mfr byte governs present/absent; the device byte
        // refines it to the expected part.
        let id = flash.read_mfr_device_id();
        if id == EXPECTED_MFR_DEVICE_ID {
            defmt::info!(
                "flash: W25Q256JV present (id {=u8:#04x} {=u8:#04x})",
                id[0],
                id[1]
            );
        } else if id[0] == EXPECTED_MFR_DEVICE_ID[0] {
            defmt::warn!(
                "flash: Winbond present, unexpected device id ({=u8:#04x} {=u8:#04x}, expected {=u8:#04x} {=u8:#04x})",
                id[0],
                id[1],
                EXPECTED_MFR_DEVICE_ID[0],
                EXPECTED_MFR_DEVICE_ID[1]
            );
        } else {
            defmt::error!(
                "flash: not responding (id {=u8:#04x} {=u8:#04x})",
                id[0],
                id[1]
            );
        }

        flash
    }
}
