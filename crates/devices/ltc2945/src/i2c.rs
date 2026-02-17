use embedded_hal::i2c::SevenBitAddress;

/// I2C interface for the LTC2945
pub struct I2cInterface<I2C> {
    /// I2c peripheral used for communication
    pub i2c: I2C,
    /// 7-bit I2C address of the device
    pub address: SevenBitAddress,
}

impl<I2C> I2cInterface<I2C> {
    /// Create a new I2C interface with the given I2C peripheral and address.
    pub const fn new(i2c: I2C, i2c_address: SevenBitAddress) -> Self {
        Self {
            i2c,
            address: i2c_address,
        }
    }

    /// Return a reference to the inner I2C interface.
    pub fn destroy(self) -> I2C {
        self.i2c
    }
}

const BUF_LEN: usize = 64;
const MAX_DATA: usize = BUF_LEN - 1;

/// Build i2c frame for writing to a register and return the length of the frame.
#[inline(always)]
fn build_frame(buf: &mut [u8; BUF_LEN], reg: u8, chunk: &[u8]) -> usize {
    // SAFETY: caller guarantees `chunk.len() ≤ MAX_DATA`
    buf[0] = reg;
    buf[1..1 + chunk.len()].copy_from_slice(chunk);
    chunk.len() + 1
}

// Asynchronous interface
impl<I2C, E> device_driver::AsyncRegisterInterface for I2cInterface<I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
{
    type Error = E;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        reg: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        if data.is_empty() {
            return Ok(());
        }

        let mut buf = [0u8; BUF_LEN];
        for chunk in data.chunks(MAX_DATA) {
            let len = build_frame(&mut buf, reg, chunk);
            self.i2c.write(self.address, &buf[..len]).await?;
        }
        Ok(())
    }

    async fn read_register(
        &mut self,
        reg: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        if data.is_empty() {
            return Ok(());
        }
        self.i2c.write_read(self.address, &[reg], data).await?;
        Ok(())
    }
}

// Synchronous interface
impl<I2C, E> device_driver::RegisterInterface for I2cInterface<I2C>
where
    I2C: embedded_hal::i2c::I2c<Error = E>,
{
    type Error = E;
    type AddressType = u8;

    fn write_register(
        &mut self,
        reg: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        if data.is_empty() {
            return Ok(());
        }

        let mut buf = [0u8; BUF_LEN];
        for chunk in data.chunks(MAX_DATA) {
            let len = build_frame(&mut buf, reg, chunk);
            self.i2c.write(self.address, &buf[..len])?;
        }
        Ok(())
    }

    fn read_register(
        &mut self,
        reg: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        if data.is_empty() {
            return Ok(());
        }
        self.i2c.write_read(self.address, &[reg], data)?;
        Ok(())
    }
}
