//! Stages each I2C transfer through a buffer in SRAM4. I2C4's only DMA is BDMA,
//! which reaches SRAM4 but not the AXI SRAM where buffers normally live, so DMA
//! straight from a normal buffer would fault.
//!
//! Not strictly needed here: validation could use blocking I2C and sidestep this
//! entirely. It exists to prototype the bounce the firmware may later adopt, so
//! this stays close to the firmware's eventual data path.

use core::sync::atomic::{AtomicBool, Ordering};

use embedded_hal::i2c::{Operation, SevenBitAddress};
use embedded_hal_async::i2c::{ErrorType, I2c};

const BOUNCE_LEN: usize = 64;

/// In SRAM4 (see `build.rs`).
#[unsafe(link_section = ".sram4")]
static mut BUFFER: [u8; BOUNCE_LEN] = [0; BOUNCE_LEN];

/// Lends the buffer out exactly once.
static TAKEN: AtomicBool = AtomicBool::new(false);

pub struct BounceI2c<I> {
    inner: I,
    buf: &'static mut [u8; BOUNCE_LEN],
}

impl<I> BounceI2c<I> {
    /// Panics if called twice; there is only one SRAM4 buffer.
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
