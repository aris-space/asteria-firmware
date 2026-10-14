// Copyright 2026 ARIS
// SPDX-License-Identifier: MIT OR Apache-2.0

#![no_std]

use embassy_stm32::Peri;
use embassy_stm32::adc::{
    Adc, AdcChannel, AdcConfig, AnyAdcChannel, Instance, RxDma, SampleTime, SpecialConverter,
    Temperature,
};
use embassy_stm32::dma;
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::pac::vrefbuf::vals::{Hiz, Vrs};

// STM32G473 internal temperature sensor factory calibration values, acquired
// at 3.0 V VDDA/VREF+ (RM0440, section 21.4.31).
// Sadly those are missing from the PAC.

#[cfg(not(feature = "stm32g473rc"))]
compile_error!("Recheck those values for another MCU");

/// Table 5 in the datasheet.
const TS_CAL1_ADDR: *const u16 = 0x1FFF_75A8 as *const u16;
const TS_CAL2_ADDR: *const u16 = 0x1FFF_75CA as *const u16;
const TS_CAL1_TEMP_C: f32 = 30.0;
const TS_CAL2_TEMP_C: f32 = 130.0;
const VDDA_CAL_VOLTAGE: f32 = 3.0;
const VREFBUF_VOLTAGE: f32 = 2.9; // at VREF2 setting

/// Marker type that guarantees that the Vrefbuf has been configured to 2.9V.
#[derive(Clone, Copy)]
pub struct ConfiguredVrefBuf(());

#[allow(non_camel_case_types)]
pub struct MCUTemperature<
    'a,
    ADC: Instance<Regs = embassy_stm32::pac::adc::Adc>,
    DMA_CH: RxDma<ADC>,
> {
    adc: Adc<'a, ADC>,
    dma: Peri<'a, DMA_CH>,
}

#[allow(non_camel_case_types)]
impl<'a, ADC, DMA_CH> MCUTemperature<'a, ADC, DMA_CH>
where
    ADC: Instance<Regs = embassy_stm32::pac::adc::Adc>,
    DMA_CH: RxDma<ADC>,
{
    /// Create a new peripheral wrapper around the ADC configured to 2.9V.
    ///
    /// Call [`setup_internal_vref_buffer`] to get the configuration marker.
    pub fn new(adc: Peri<'a, ADC>, dma: Peri<'a, DMA_CH>, _cfg: ConfiguredVrefBuf) -> Self {
        let adc = Adc::new(adc, AdcConfig::default());
        Self { adc, dma }
    }

    /// Reads the internal temperature sensor of the MCU.
    ///
    /// Returns the temperature in degrees celsius.
    pub async fn read_internal_temperature(
        &mut self,
        irq: impl Binding<DMA_CH::Interrupt, dma::InterruptHandler<DMA_CH>>,
    ) -> f32
    where
        ADC: SpecialConverter<Temperature>,
    {
        let mut temp = self.adc.enable_temperature();
        let mut pin = temp.degrade_adc();
        let raw = Self::read_raw_static(&mut self.adc, self.dma.reborrow(), &mut pin, irq).await;

        // SAFETY: TS_CAL1/2 are factory-programmed 16-bit words at fixed system memory addresses.
        let ts_cal1 = unsafe { TS_CAL1_ADDR.read_volatile() } as f32;
        let ts_cal2 = unsafe { TS_CAL2_ADDR.read_volatile() } as f32;

        // The calibration words were captured at VDDA = VDDA_CAL_VOLTAGE, but
        // we are actually sampling with VREF+ driven to VREFBUF_VOLTAGE.
        // We guarantee that setup by taking `ConfiguredVrefBuf` in the constructor
        let raw_scaled = raw as f32 * (VREFBUF_VOLTAGE / VDDA_CAL_VOLTAGE);

        // Linear interpolation between the two factory calibration points.
        (raw_scaled - ts_cal1) * (TS_CAL2_TEMP_C - TS_CAL1_TEMP_C) / (ts_cal2 - ts_cal1)
            + TS_CAL1_TEMP_C
    }

    async fn read_raw_static(
        adc: &mut Adc<'_, ADC>,
        dma: Peri<'_, DMA_CH>,
        pin: &mut AnyAdcChannel<'_, ADC>,
        irq: impl Binding<DMA_CH::Interrupt, dma::InterruptHandler<DMA_CH>>,
    ) -> u16 {
        let mut read_buf = [0; 1];
        adc.read(
            dma,
            irq,
            [(pin, SampleTime::CYCLES640_5)].into_iter(),
            &mut read_buf,
        )
        .await;

        read_buf[0]
    }
}

/// This function synchronously enables the VREFBUF, which drives VRef+ to 2V9
///
/// # Safety
/// This is only safe to call if the VREF+ pin is not connected to any other voltage,
/// otherwise it shorts the MCU.
/// Additionally, after calling this, you must not manually reconfigure the Vrefbus,
/// otherwise the resulting measurements will be off.
pub unsafe fn setup_internal_vref_buffer() -> ConfiguredVrefBuf {
    use embassy_stm32::pac::VREFBUF;

    let csr = VREFBUF.csr();

    csr.modify(|csr| {
        csr.set_vrs(Vrs::VREF2); // 2.9V
        csr.set_envr(true); // enable voltage reference
        csr.set_hiz(Hiz::CONNECTED); // connect to Vref+
    });

    while !csr.read().vrr() {
        // Wait for the VREFBUF to be ready
        cortex_m::asm::nop();
    }

    ConfiguredVrefBuf(())
}
