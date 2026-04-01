#![no_std]
#![no_main]
#![allow(non_camel_case_types)]
#![allow(dead_code)]
pub mod pressures;
use crate::pressures::{TrafagPSens, VOLTAGE_RANGE};
use embassy_stm32::adc::{
    Adc, AdcChannel, AdcConfig, AnyAdcChannel, Instance, RxDma, SampleTime, SpecialConverter,
    Temperature, VrefInt,
};
use embassy_stm32::dma;
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::pac::vrefbuf::vals::{Hiz, Vrs};
use embassy_stm32::rcc::{Sysclk, mux};
use embassy_stm32::{Config, Peri, PeripheralType};
use embedded_utils::info;

// Voltage the VREFINT reg was calibrated at the factory
const VREFBUF_CALIB: f32 = 3.0;

const ADC_CALIBRATION_SAMPLES: u64 = 50;

pub struct ADCPressure<'a, ADC: Instance<Regs = embassy_stm32::pac::adc::Adc>, DMA_CH: RxDma<ADC>> {
    adc: Adc<'a, ADC>,
    dma: Peri<'a, DMA_CH>,
    sensor: TrafagPSens<'a, ADC>,
    pub vref_calib: f32,
}

impl<'a, ADC, DMA_CH> ADCPressure<'a, ADC, DMA_CH>
where
    ADC: Instance<Regs = embassy_stm32::pac::adc::Adc>,
    DMA_CH: RxDma<ADC>,
{
    pub async fn new(adc: Peri<'a, ADC>, dma: Peri<'a, DMA_CH>, sensor: TrafagPSens<'a, ADC>) -> Self {
        let adc = Adc::new(adc, AdcConfig::default());

        Self {
            adc,
            dma,
            sensor,
            vref_calib: 0.0,
        }
    }

    pub async fn calibrate(&mut self, n_samples: u64, irq: impl Binding<DMA_CH::Interrupt, dma::InterruptHandler<DMA_CH>> + Copy)
    where
        ADC: SpecialConverter<VrefInt>,
    {
        let mut vref = 0.0;
        for _ in 0..n_samples {
            vref += self.read_vref_int(irq).await;
        }
        vref /= n_samples as f32;
        self.vref_calib = vref;

        info!("[ADC] VREF used: {}", self.vref_calib);
    }

    pub async fn read_internal_temperature(&mut self, irq: impl Binding<DMA_CH::Interrupt, dma::InterruptHandler<DMA_CH>> + Copy) -> f32
    where
        ADC: SpecialConverter<Temperature>,
    {
        let ts_cal1 = unsafe { core::ptr::read_volatile(0x1FFF_75A8 as *const u16) } as i16 as f32;
        let ts_cal2 = unsafe { core::ptr::read_volatile(0x1FFF_75CA as *const u16) } as i16 as f32;

        let mut temp = self.adc.enable_temperature();
        let mut pin = temp.degrade_adc();
        let raw = Self::read_raw_static(&mut self.adc, self.dma.reborrow(), &mut pin, irq).await;
        (130.0 - 30.0) / (ts_cal2 - ts_cal1) * (raw as i16 as f32 * self.vref_calib / 3.0)
    }

    pub async fn read_vref_int(&mut self, irq: impl Binding<DMA_CH::Interrupt, dma::InterruptHandler<DMA_CH>> + Copy) -> f32
    where
        ADC: SpecialConverter<VrefInt>,
    {
        // VREFBUF calibration values
        let vref_cal = unsafe { core::ptr::read_volatile(0x1FFF_75AA as *const u16) };
        let mut vref = self.adc.enable_vrefint();
        let mut pin = vref.degrade_adc();
        let raw = Self::read_raw_static(&mut self.adc, self.dma.reborrow(), &mut pin, irq).await;

        VREFBUF_CALIB * vref_cal as i16 as f32 / raw as i16 as f32
    }

    pub async fn read_pressure(&mut self, irq: impl Binding<DMA_CH::Interrupt, dma::InterruptHandler<DMA_CH>> + Copy) -> f32 {
        let raw = Self::read_raw_static(&mut self.adc, self.dma.reborrow(), &mut self.sensor.pin, irq).await;
        let voltage = (raw as i16 as f32) * self.vref_calib / 4095.0;

        self.sensor.si_range[0]
            + (self.sensor.si_range[1] - self.sensor.si_range[0])
                / (VOLTAGE_RANGE[1] - VOLTAGE_RANGE[0])
                * (voltage - VOLTAGE_RANGE[0])
    }

    async fn read_raw_static(adc: &mut Adc<'_, ADC>, dma: Peri<'_, DMA_CH>, pin: &mut AnyAdcChannel<'_, ADC>, irq: impl Binding<DMA_CH::Interrupt, dma::InterruptHandler<DMA_CH>>) -> u16 {
        let mut read_buf = [0; 1];
        adc.read(
                dma,
                irq,
                [(pin, SampleTime::CYCLES247_5)].into_iter(),
                &mut read_buf,
            )
            .await;

        read_buf[0]
    }
}

pub fn config_vref_buf() {
    use embassy_stm32::pac::VREFBUF;

    let csr = VREFBUF.csr();

    csr.modify(|csr| {
        csr.set_vrs(Vrs::VREF0);

        csr.set_envr(true);
        csr.set_hiz(Hiz::CONNECTED);
    });

    info!(
        "VREFBUF Config: ENVR: {}, HIZ: {}, VRS: {}",
        csr.read().envr(),
        csr.read().hiz().to_bits(),
        csr.read().vrs().to_bits()
    );

    while !csr.read().vrr() {
        // Wait for the VREFBUF to be ready
        cortex_m::asm::nop();
    }
}

pub fn set_adc_configs(config: &mut Config) {
    config.rcc.mux.adc12sel = mux::Adcsel::SYS;
    config.rcc.mux.adc345sel = mux::Adcsel::SYS;
    config.rcc.sys = Sysclk::HSE;
}
