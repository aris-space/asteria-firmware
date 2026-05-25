//! I2C4 lives in the D3 domain, so its DMA is BDMA, and BDMA can only reach D3
//! RAM (SRAM4) — not the AXI SRAM where buffers normally live. [`BounceI2c`] wraps
//! such a bus and stages every transfer through a buffer in SRAM4, so the DMA only
//! ever touches D3 RAM and the async sensor drivers run unchanged.
//!
//! Wrap a bus once and share it behind a mutex: the SRAM4 buffer is handed out
//! exactly once (so a second `BounceI2c` cannot exist), and the sharing mutex
//! serialises every transfer through it.

use core::sync::atomic::{AtomicBool, Ordering};

use embedded_hal::i2c::{Operation, SevenBitAddress};
use embedded_hal_async::i2c::{ErrorType, I2c};

/// The largest single transfer the on-board sensors perform is a handful of
/// bytes; 64 is comfortable headroom.
const BOUNCE_LEN: usize = 64;

/// Bounce buffer placed in SRAM4 by the linker (see `build.rs`).
#[unsafe(link_section = ".sram4")]
static mut BUFFER: [u8; BOUNCE_LEN] = [0; BOUNCE_LEN];

/// Guards the SRAM4 buffer so it is lent out exactly once.
static TAKEN: AtomicBool = AtomicBool::new(false);

/// Wraps an I2C bus whose DMA is BDMA, staging transfers through SRAM4.
pub struct BounceI2c<I> {
    inner: I,
    buf: &'static mut [u8; BOUNCE_LEN],
}

impl<I> BounceI2c<I> {
    /// Wrap `inner`. Panics if called more than once: there is a single SRAM4
    /// bounce buffer, hence a single BDMA bus.
    pub fn new(inner: I) -> Self {
        assert!(
            !TAKEN.swap(true, Ordering::AcqRel),
            "BounceI2c: the SRAM4 bounce buffer is already in use"
        );
        // SAFETY: TAKEN guarantees this &mut is unique; BUFFER lives in SRAM4.
        let buf = unsafe { &mut *core::ptr::addr_of_mut!(BUFFER) };
        Self { inner, buf }
    }
}

impl<I: ErrorType> ErrorType for BounceI2c<I> {
    type Error = I::Error;
}

impl<I: I2c<SevenBitAddress>> I2c<SevenBitAddress> for BounceI2c<I> {
    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        assert!(
            read.len() <= BOUNCE_LEN,
            "BounceI2c: read exceeds SRAM4 buffer"
        );
        let buf = &mut self.buf[..read.len()];
        self.inner.read(address, buf).await?;
        read.copy_from_slice(buf);
        Ok(())
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        assert!(
            write.len() <= BOUNCE_LEN,
            "BounceI2c: write exceeds SRAM4 buffer"
        );
        let buf = &mut self.buf[..write.len()];
        buf.copy_from_slice(write);
        self.inner.write(address, buf).await
    }

    async fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), Self::Error> {
        assert!(
            write.len() + read.len() <= BOUNCE_LEN,
            "BounceI2c: write_read exceeds SRAM4 buffer"
        );
        let (wbuf, rest) = self.buf.split_at_mut(write.len());
        wbuf.copy_from_slice(write);
        let rbuf = &mut rest[..read.len()];
        self.inner.write_read(address, wbuf, rbuf).await?;
        read.copy_from_slice(rbuf);
        Ok(())
    }

    async fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        // The sensor drivers here only use read/write/write_read, so each operation
        // is bounced on its own; this is not one repeated-start transaction.
        for op in operations {
            match op {
                Operation::Read(read) => {
                    assert!(
                        read.len() <= BOUNCE_LEN,
                        "BounceI2c: read exceeds SRAM4 buffer"
                    );
                    let buf = &mut self.buf[..read.len()];
                    self.inner.read(address, buf).await?;
                    read.copy_from_slice(buf);
                }
                Operation::Write(write) => {
                    assert!(
                        write.len() <= BOUNCE_LEN,
                        "BounceI2c: write exceeds SRAM4 buffer"
                    );
                    let buf = &mut self.buf[..write.len()];
                    buf.copy_from_slice(write);
                    self.inner.write(address, buf).await?;
                }
            }
        }
        Ok(())
    }
}
