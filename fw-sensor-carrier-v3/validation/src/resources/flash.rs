//! On-board W25Q256JV NOR flash on OCTOSPI1, operated in plain single-line SPI
//! (1-1-1) mode. IO2 (/WP) and IO3 (/HOLD) are held high as GPIOs so the part
//! behaves like a classic SPI flash. Mirrors the firmware's flash bring-up; the
//! validation only needs the JEDEC presence probe.

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::ospi::{Config as OspiConfig, MemorySize, Ospi, OspiWidth, TransferConfig};
use embassy_stm32::peripherals::OCTOSPI1;

use super::Flash;

/// JEDEC manufacturer byte a healthy W25Q256JV returns: Winbond.
pub const WINBOND_MANUFACTURER_ID: u8 = 0xEF;

const READ_JEDEC_ID: u8 = 0x9F;

pub struct BoardFlash {
    ospi: Ospi<'static, OCTOSPI1, Blocking>,
    _hold: Output<'static>,
    _wp: Output<'static>,
}

impl BoardFlash {
    /// Read the JEDEC manufacturer byte. On this instruction-then-data OSPI path
    /// the continuation bytes mis-clock, so only byte 0 (the manufacturer) is
    /// trustworthy; that is enough for a "chip present" probe.
    pub fn manufacturer_id(&mut self) -> u8 {
        let mut id = [0u8; 1];
        let _ = self.ospi.blocking_read(
            &mut id,
            TransferConfig {
                iwidth: OspiWidth::SING,
                instruction: Some(READ_JEDEC_ID as u32),
                dwidth: OspiWidth::SING,
                ..Default::default()
            },
        );
        id[0]
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
            config(),
        );

        BoardFlash {
            ospi,
            _hold: hold,
            _wp: wp,
        }
    }
}
