#![no_std]

mod device;
mod i2c;

use device::Ina232device;
use device_driver::{AsyncRegisterInterface, RegisterInterface};
use embedded_hal::i2c::SevenBitAddress;

pub use i2c::I2cInterface;

/// Calibration value for 5mΩ shunt with Current_LSB = 1mA
const SHUNT_CAL_5M: u16 = 1024;

/// A driver for the INA232
pub struct Ina232<I> {
    inner: Ina232device<I>,
}

impl<I2C> Ina232<I2cInterface<I2C>> {
    /// Create a new instance of the INA232 driver using I2C.
    /// Writes the calibration register for a 5mΩ shunt.
    pub fn new_i2c(i2c: I2C, address: SevenBitAddress) -> Self {
        let interface = I2cInterface::new(i2c, address);
        let inner = Ina232device::new(interface);
        Self { inner }
    }
}

impl<I> Ina232<I> {
    /// Create a new instance of the INA232 driver.
    pub const fn new(interface: I) -> Self {
        let inner = Ina232device::new(interface);
        Self { inner }
    }

    /// Get a reference to the inner device.
    pub const fn inner(&self) -> &Ina232device<I> {
        &self.inner
    }

    /// Get a mutable reference to the inner device.
    pub const fn inner_mut(&mut self) -> &mut Ina232device<I> {
        &mut self.inner
    }

    /// Consume the driver and return the inner interface.
    pub fn destroy(self) -> I {
        self.inner.interface
    }
}

impl<I, E> Ina232<I>
where
    I: AsyncRegisterInterface<Error = E, AddressType = u8>,
{
    /// Initialize the device by writing the calibration register.
    /// Must be called once before reading current or power.
    pub async fn init(&mut self) -> Result<(), E> {
        self.inner
            .calibration()
            .write_async(|reg| reg.set_shunt_cal(SHUNT_CAL_5M))
            .await
    }

    /// Async read bus voltage in volts.
    pub async fn read_voltage(&mut self) -> Result<f32, E> {
        let reg = self.inner.bus_voltage().read_async().await?;
        let raw: u16 = reg.vbus();
        Ok((raw as f32) * 0.0016) // 1.6mV per LSB
    }

    /// Async read current in amperes.
    pub async fn read_current(&mut self) -> Result<f32, E> {
        let reg = self.inner.current().read_async().await?;
        let raw: i16 = reg.current();
        Ok((raw as f32) * 0.001) // Current_LSB = 1mA
    }

    /// Async read power in watts.
    pub async fn read_power(&mut self) -> Result<f32, E> {
        let reg = self.inner.power().read_async().await?;
        let raw: u16 = reg.power();
        Ok((raw as f32) * 0.032) // 32 * Current_LSB
    }
}

impl<I, E> Ina232<I>
where
    I: RegisterInterface<Error = E, AddressType = u8>,
{
    /// Blocking initialize the device by writing the calibration register.
    pub fn blocking_init(&mut self) -> Result<(), E> {
        self.inner
            .calibration()
            .write(|reg| reg.set_shunt_cal(SHUNT_CAL_5M))
    }

    /// Blocking read bus voltage in volts.
    pub fn blocking_read_voltage(&mut self) -> Result<f32, E> {
        let reg = self.inner.bus_voltage().read()?;
        let raw: u16 = reg.vbus();
        Ok((raw as f32) * 0.0016)
    }

    /// Blocking read current in amperes.
    pub fn blocking_read_current(&mut self) -> Result<f32, E> {
        let reg = self.inner.current().read()?;
        let raw: i16 = reg.current();
        Ok((raw as f32) * 0.001)
    }

    /// Blocking read power in watts.
    pub fn blocking_read_power(&mut self) -> Result<f32, E> {
        let reg = self.inner.power().read()?;
        let raw: u16 = reg.power();
        Ok((raw as f32) * 0.032)
    }
}
