#![no_std]

mod device;
mod i2c;

use device::Ltc2945device;
use device_driver::{AsyncRegisterInterface, RegisterInterface};
use embedded_hal::i2c::SevenBitAddress;

pub use i2c::I2cInterface;

/// A driver for the LTC2945
pub struct Ltc2945<I> {
    inner: Ltc2945device<I>,
}

impl<I2C> Ltc2945<I2cInterface<I2C>> {
    /// Create a new instance of the LTC2945 driver using I2C.
    pub const fn new_i2c(i2c: I2C, address: SevenBitAddress) -> Self {
        let interface = I2cInterface::new(i2c, address);
        let inner = Ltc2945device::new(interface);
        Self { inner }
    }
}

impl<I> Ltc2945<I> {
    /// Create a new instance of the LTC2945 driver.
    pub const fn new(interface: I) -> Self {
        let inner = Ltc2945device::new(interface);
        Self { inner }
    }

    /// Get a reference to the inner device.
    pub const fn inner(&self) -> &Ltc2945device<I> {
        &self.inner
    }

    /// Get a mutable reference to the inner device.
    pub const fn inner_mut(&mut self) -> &mut Ltc2945device<I> {
        &mut self.inner
    }

    /// Consume the driver and return the inner interface.
    pub fn destroy(self) -> I {
        self.inner.interface
    }
}

impl<I, E> Ltc2945<I>
where
    I: AsyncRegisterInterface<Error = E, AddressType = u8>,
{
    /// Async read VIN and convert to voltage in volts.
    pub async fn read_voltage(&mut self) -> Result<f32, E> {
        // Read 16-bit VIN register (MSB at 0x1E, LSB at 0x1F)
        let reg = self.inner.vin().read_async().await?;
        let raw: u16 = reg.vin(); // 16-bit raw, lower 4 bits are unused
        let value = raw >> 4; // keep only upper 12 bits
        Ok((value as f32) * (102.4 / 4096.0))
    }

    /// Async read ΔSENSE and convert to current in amperes.
    pub async fn read_current(&mut self) -> Result<f32, E> {
        // Read the 16-bit ΔSENSE register (MSB at 0x1A, LSB at 0x1B)
        let reg = self.inner.delta_sense().read_async().await?;
        let raw: u16 = reg.delta_sense() >> 4; // keep only the upper 12 bits
        Ok((raw as f32) * 0.005) // 25μ/5m = 0.005
    }

    /// Async read power from the device and convert to watts.
    pub async fn read_power(&mut self) -> Result<f32, E> {
        let reg = self.inner.power().read_async().await?;
        let raw: u32 = reg.power(); // 24-bit raw power count
        Ok((raw as f32) * 0.000125)
    }
}

impl<I, E> Ltc2945<I>
where
    I: RegisterInterface<Error = E, AddressType = u8>,
{
    /// Blocking read VIN and convert to voltage in volts.
    pub fn blocking_read_voltage(&mut self) -> Result<f32, I::Error> {
        // Read 16-bit VIN register (MSB at 0x1E, LSB at 0x1F)
        let reg = self.inner.vin().read()?;
        let raw: u16 = reg.vin(); // 16-bit raw, lower 4 bits are unused
        let value = raw >> 4; // keep only upper 12 bits
        Ok((value as f32) * (102.4 / 4096.0))
    }

    /// Blocking read ΔSENSE and convert to current in amperes.
    pub fn blocking_read_current(&mut self) -> Result<f32, E> {
        // Read the 16-bit ΔSENSE register (MSB at 0x1A, LSB at 0x1B)
        let reg = self.inner.delta_sense().read()?;
        let raw: u16 = reg.delta_sense() >> 4; // keep only the upper 12 bits
        Ok((raw as f32) * 0.005) // 25μ/5m = 0.005
    }

    /// Blocking read power from the device and convert to watts.
    pub fn blocking_read_power(&mut self) -> Result<f32, E> {
        let reg = self.inner.power().read()?;
        let raw: u32 = reg.power(); // 24-bit raw power count
        Ok((raw as f32) * 0.000125)
    }
}
