const READ_BIT: u8 = 0x80;

pub struct Lsm6Dso32SpiInterface<SPI> {
    pub spi: SPI,
}

impl<SPI, E> device_driver::AsyncRegisterInterface for Lsm6Dso32SpiInterface<SPI>
where
    SPI: embedded_hal_async::spi::SpiDevice<u8, Error = E>,
{
    type Error = E;
    type AddressType = u8;

    async fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [
                embedded_hal_async::spi::Operation::Write(&[address & !READ_BIT]),
                embedded_hal_async::spi::Operation::Write(data),
            ])
            .await?;

        Ok(())
    }

    async fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.spi
            .transaction(&mut [
                embedded_hal_async::spi::Operation::Write(&[address | READ_BIT]),
                embedded_hal_async::spi::Operation::Read(data),
            ])
            .await?;

        Ok(())
    }
}

impl<SPI, E> device_driver::RegisterInterface for Lsm6Dso32SpiInterface<SPI>
where
    SPI: embedded_hal::spi::SpiDevice<u8, Error = E>,
{
    type Error = E;
    type AddressType = u8;

    fn write_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &[u8],
    ) -> Result<(), Self::Error> {
        /*#[cfg(feature = "defmt-03")]
        trace!(
            "LSM6DSO32 SPI write register, address: {:#X}, data: {:?}",
            address, data
        );
         */

        self.spi.transaction(&mut [
            embedded_hal::spi::Operation::Write(&[address & !READ_BIT]),
            embedded_hal::spi::Operation::Write(data),
        ])?;

        Ok(())
    }

    fn read_register(
        &mut self,
        address: Self::AddressType,
        _size_bits: u32,
        data: &mut [u8],
    ) -> Result<(), Self::Error> {
        /*#[cfg(feature = "defmt-03")]
        trace!(
            "LSM6DSO32 SPI read register, address: {:#X}, data: {:?}",
            address, data
        );
         */

        self.spi.transaction(&mut [
            embedded_hal::spi::Operation::Write(&[address | READ_BIT]),
            embedded_hal::spi::Operation::Read(data),
        ])?;

        Ok(())
    }
}
