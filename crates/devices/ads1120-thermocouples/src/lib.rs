// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]
#![allow(dead_code)]
#![allow(clippy::too_many_arguments)]
pub mod thermocouple_conversions;

use crate::ADSError::SpiError;
use core::slice;
use embassy_stm32::dma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::mode::Async;
use embassy_stm32::spi::mode::Master;
use embassy_stm32::spi::{BitOrder, Error, MisoPin, MosiPin, RxDma, SckPin, Spi, TxDma};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Peri, spi};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_utils::{error, trace};

const READ_TIME_OUT: Duration = Duration::from_millis(500); // 500 ms

const ADC_VREF_INT: f32 = 2048.0; // Internal reference voltage in mV
const ADC_FULL_SCALE_MAX: f32 = 65536.0; // 2^16-1

enum ADSCmd {
    Reset = 0x06,
    StartSync = 0x08,
    PowerDown = 0x02,
    ReadData = 0x10,
    ReadReg = 0x20,
    WriteReg = 0x40,
}

#[repr(u8)]
pub enum PGAGain {
    Gain1 = 0x00,
    Gain2 = 0x01,
    Gain4 = 0x02,
    Gain8 = 0x03,
    Gain16 = 0x04,
    Gain32 = 0x05,
    Gain64 = 0x06,
    Gain128 = 0x07,
}

impl PGAGain {
    fn to_value(&self) -> f32 {
        let val = match self {
            PGAGain::Gain1 => 1.0,
            PGAGain::Gain2 => 2.0,
            PGAGain::Gain4 => 4.0,
            PGAGain::Gain8 => 8.0,
            PGAGain::Gain16 => 16.0,
            PGAGain::Gain32 => 32.0,
            PGAGain::Gain64 => 64.0,
            PGAGain::Gain128 => 128.0,
        };
        val / 2.0 // Divide by 2 to get the actual gain value
        // This was needed because for some reason when using the actual gain values the voltage readings were off by a factor of 2
        // Never found the reason why, so this is a workaround
    }

    fn to_byte(&self) -> u8 {
        match self {
            PGAGain::Gain1 => 0,
            PGAGain::Gain2 => 1,
            PGAGain::Gain4 => 2,
            PGAGain::Gain8 => 3,
            PGAGain::Gain16 => 4,
            PGAGain::Gain32 => 5,
            PGAGain::Gain64 => 6,
            PGAGain::Gain128 => 7,
        }
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ADSError {
    RegisterSetError,
    ValueRangeError,
    ResetError,
    ReadTimeOut,
    SpiError(Error),
}

pub struct ADSThermocouples<'a> {
    handle: Spi<'a, Async, Master>,
    ext_irq: ExtiInput<'a, Async>,
    voltage_offset: f32,
    pga_gain: PGAGain,
    _cs: Output<'a>,
}

impl<'a> ADSThermocouples<'a> {
    pub async fn new<T: spi::Instance, D1: TxDma<T>, D2: RxDma<T>>(
        peri: Peri<'a, T>,
        sck: Peri<'a, impl SckPin<T>>,
        mosi: Peri<'a, impl MosiPin<T>>,
        miso: Peri<'a, impl MisoPin<T>>,
        tx_dma: Peri<'a, D1>,
        rx_dma: Peri<'a, D2>,
        irq: impl Binding<D1::Interrupt, dma::InterruptHandler<D1>>
        + Binding<D2::Interrupt, dma::InterruptHandler<D2>>
        + 'a,
        cs: Peri<'a, impl Pin>,
        ext_irq: ExtiInput<'a, Async>,
        pga_gain: PGAGain,
    ) -> Result<Self, ADSError> {
        // Set config
        let mut config = spi::Config::default();
        config.mode = spi::MODE_1;
        config.bit_order = BitOrder::MsbFirst;
        config.frequency = Hertz(1_000_000);

        // Initialize SPI
        let handle = Spi::new(peri, sck, mosi, miso, tx_dma, rx_dma, irq, config);
        // Set CS pin
        let mut cs = Output::new(cs, Level::High, Speed::Low);
        cs.set_low();

        let mut tcs = ADSThermocouples {
            handle,
            ext_irq,
            voltage_offset: 0.0,
            pga_gain,
            _cs: cs,
        };

        // Reset the device
        tcs.reset().await?;
        trace!("ADS1120 Reset");

        // Set General Config Params
        tcs.set_general_configs(
            0x00,  // Data Rate, 20 Hz
            0x00,  // Mode, Normal
            false, // Single Shot mode
            false, // Temperature sensor
            false, // BCS, sensor detection disabled
            0x00,  // Vref, internal
            0x00,  // 50/60Hz, internal filtering disabled
            false, // PSW, power switch, disabled
            0x00,  // IDAC, current source, disabled
            0x00,  // I1MUX, disabled
            0x00,  // I2MUX, disabled
            false, // DRDY mode, activated
        )
        .await?;
        trace!("ADS1120 Configured, calibrating...");

        // Calibrate offset
        let n_samples = 50;
        tcs.calibrate_offset(n_samples).await?;
        trace!("ADS1120 calibrated, offset: {}mV", tcs.voltage_offset);

        Ok(tcs)
    }

    pub async fn read_primary(&mut self) -> Result<f32, ADSError> {
        // Set the config for the primary thermocouple
        self.set_primary_config().await?;
        // Start the conversion
        self.start_sync().await?;

        // Wait for the conversion to complete
        with_timeout(READ_TIME_OUT, self.ext_irq.wait_for_falling_edge())
            .await
            .map_err(|_| ADSError::ReadTimeOut)?;

        self.read_data().await
    }

    pub async fn read_secondary(&mut self) -> Result<f32, ADSError> {
        // Set the config for the secondary thermocouple
        self.set_secondary_config().await?;
        // Start the conversion
        self.start_sync().await?;

        // Wait for the conversion to complete
        with_timeout(READ_TIME_OUT, self.ext_irq.wait_for_falling_edge())
            .await
            .map_err(|_| ADSError::ReadTimeOut)?;

        self.read_data().await
    }

    async fn set_primary_config(&mut self) -> Result<(), ADSError> {
        // Set the config for the primary thermocouple
        self.set_config_reg0(
            0x00,  // MUX, set to AIN0, AIN1
            false, // PGA Bypass, disabled
        )
        .await?;
        Ok(())
    }

    async fn set_secondary_config(&mut self) -> Result<(), ADSError> {
        // Set the config for the secondary thermocouple
        self.set_config_reg0(
            0x05,  // MUX, set to AIN2, AIN3
            false, // PGA Bypass, disabled
        )
        .await?;
        Ok(())
    }

    async fn set_general_configs(
        &mut self,
        dr: u8,
        mode: u8,
        cm: bool,
        ts: bool,
        bcs: bool,
        vref: u8,
        fifty_sixty: u8,
        psw: bool,
        idac: u8,
        i1mux: u8,
        i2mux: u8,
        drdym: bool,
    ) -> Result<(), ADSError> {
        let reg1_byte =
            (dr & 0x07) << 5 | (mode & 0x03) << 3 | (cm as u8) << 2 | (ts as u8) << 1 | (bcs as u8);
        self.write_reg_single(0x01, reg1_byte).await?;

        let reg2_byte =
            (vref & 0x03) << 6 | (fifty_sixty & 0x03) << 3 | (psw as u8) << 2 | (idac & 0x07);
        self.write_reg_single(0x02, reg2_byte).await?;

        let reg3_byte = (i1mux & 0x07) << 5 | (i2mux & 0x07) << 2 | (drdym as u8) << 1;
        self.write_reg_single(0x03, reg3_byte).await?;

        Ok(())
    }

    async fn set_config_reg0(&mut self, mux: u8, pga_bypass: bool) -> Result<(), ADSError> {
        let reg0_byte =
            (mux & 0x0F) << 4 | (self.pga_gain.to_byte() & 0x07) << 1 | (pga_bypass as u8);
        self.write_reg_single(0x00, reg0_byte).await?;

        Ok(())
    }

    pub async fn calibrate_offset(&mut self, n_samples: u32) -> Result<(), ADSError> {
        // Set config for calibration
        // set mux to short AINs to (AVDD + AVSS) / 2

        let mut offset: f32 = 0.0;
        self.set_config_reg0(0x0E, false).await?;

        self.voltage_offset = 0.0;
        for _ in 0..n_samples {
            // Start the conversion
            self.start_sync().await?;

            // Wait for the conversion to complete
            with_timeout(READ_TIME_OUT, self.ext_irq.wait_for_falling_edge())
                .await
                .map_err(|_| ADSError::ReadTimeOut)?;

            let voltage_one_time = self.read_data().await?;
            // Read the data
            offset += voltage_one_time;
        }
        // Calculate the average offset
        self.voltage_offset = offset / n_samples as f32;

        Ok(())
    }

    ////////////////////// Commands /////////////////////
    async fn reset(&mut self) -> Result<(), ADSError> {
        // Reset the device
        self.handle
            .write(&[ADSCmd::Reset as u8])
            .await
            .map_err(SpiError)?;
        // Wait for the device to be ready
        with_timeout(READ_TIME_OUT, self.ext_irq.wait_for_falling_edge())
            .await
            .map_err(|_| ADSError::ReadTimeOut)?;
        Ok(())
    }

    async fn start_sync(&mut self) -> Result<(), ADSError> {
        // Start internal ADC conversion
        self.handle
            .write(&[ADSCmd::StartSync as u8])
            .await
            .map_err(SpiError)?;
        Ok(())
    }

    async fn power_down(&mut self) -> Result<(), ADSError> {
        // Power down the device
        self.handle
            .write(&[ADSCmd::PowerDown as u8])
            .await
            .map_err(SpiError)?;
        Ok(())
    }

    async fn read_data(&mut self) -> Result<f32, ADSError> {
        // Write command
        self.handle
            .write(&[ADSCmd::ReadData as u8])
            .await
            .map_err(SpiError)?;

        // Read data
        let mut adc_value = 0u16;
        self.handle
            .read(slice::from_mut(&mut adc_value))
            .await
            .map_err(SpiError)?;

        // Convert to voltage
        let voltage = (ADC_VREF_INT / ADC_FULL_SCALE_MAX) * (adc_value as i16 as f32)
            / self.pga_gain.to_value()
            - self.voltage_offset;

        Ok(voltage)
    }

    async fn read_reg_single(&mut self, reg_offset: u8) -> Result<u8, ADSError> {
        let cmd = (ADSCmd::ReadReg as u8) | ((reg_offset & 0x03) << 2);

        // Write command
        self.handle
            .write(slice::from_ref(&cmd))
            .await
            .map_err(SpiError)?;

        // Read data
        let mut byte = 0u8;
        self.handle
            .read(slice::from_mut(&mut byte))
            .await
            .map_err(SpiError)?;

        Ok(byte)
    }

    async fn write_reg_single(&mut self, reg_offset: u8, byte: u8) -> Result<(), ADSError> {
        let cmd = (ADSCmd::WriteReg as u8) | ((reg_offset & 0x03) << 2);

        // Write command
        self.handle
            .write(slice::from_ref(&cmd))
            .await
            .map_err(SpiError)?;

        // Write data
        self.handle
            .write(slice::from_ref(&byte))
            .await
            .map_err(SpiError)?;

        Timer::after_micros(50).await;

        // Read back the register to verify
        let reg = self.read_reg_single(reg_offset).await?;

        if reg != byte {
            error!(
                "Register set error: expected {:#04b}, got {:#04b}",
                byte, reg
            );
            return Err(ADSError::RegisterSetError);
        }

        Ok(())
    }
}
