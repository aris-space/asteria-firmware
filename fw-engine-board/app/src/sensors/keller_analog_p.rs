use crate::Irqs;
use crate::drivers::analog_pressure::{EnginePressureDriver, EnginePressureMeasurementRaw};
use crate::sensors::{
    ACQ_PRESSURE_FREQ_HZ, ADC_CALIBRATION_SAMPLES, ENG_CC_P_RANGE, FUE_INJ_P_RANGE, OXD_INJ_P_RANGE,
};
use analog_pressure::ADCPressure;
use analog_pressure::pressures::TrafagPSens;
use embassy_futures::join::join3;
use embassy_stm32::Peri;
use embassy_stm32::adc::AdcChannel;
use embassy_stm32::peripherals::{ADC1, ADC2, ADC3, DMA1_CH4, DMA2_CH2, DMA2_CH3, PB13, PC0, PC1};
use embassy_time::{Duration, Ticker};
use embedded_utils::info;

pub struct EnginePressureHandles {
    pub eng_cc_p_adc: Peri<'static, ADC1>,
    pub eng_cc_p_dma: Peri<'static, DMA1_CH4>,
    pub eng_cc_p_pin: Peri<'static, PC0>,
    pub eng_inj_p_adc: Peri<'static, ADC2>,
    pub eng_inj_p_dma: Peri<'static, DMA2_CH3>,
    pub eng_inj_p_pin: Peri<'static, PC1>,
    pub oss_inj_p_adc: Peri<'static, ADC3>,
    pub oss_inj_p_dma: Peri<'static, DMA2_CH2>,
    pub oss_inj_p_pin: Peri<'static, PB13>,
}

#[embassy_executor::task]
pub async fn engine_pressure_acquisition(pressure_handles: EnginePressureHandles) {
    let mut data_publisher = EnginePressureDriver::new();

    let mut eng_cc_p_pin = pressure_handles.eng_cc_p_pin;
    let mut eng_inj_p_pin = pressure_handles.eng_inj_p_pin;
    let mut oss_inj_p_pin = pressure_handles.oss_inj_p_pin;

    let mut eng_cc_p_handle = ADCPressure::new(
        pressure_handles.eng_cc_p_adc,
        pressure_handles.eng_cc_p_dma,
        TrafagPSens {
            pin: eng_cc_p_pin.degrade_adc(),
            si_range: ENG_CC_P_RANGE,
        },
    )
    .await;

    let mut eng_inj_p_handle = ADCPressure::new(
        pressure_handles.eng_inj_p_adc,
        pressure_handles.eng_inj_p_dma,
        TrafagPSens {
            pin: eng_inj_p_pin.degrade_adc(),
            si_range: FUE_INJ_P_RANGE,
        },
    )
    .await;

    let mut oss_inj_p_handle = ADCPressure::new(
        pressure_handles.oss_inj_p_adc,
        pressure_handles.oss_inj_p_dma,
        TrafagPSens {
            pin: oss_inj_p_pin.degrade_adc(),
            si_range: OXD_INJ_P_RANGE,
        },
    )
    .await;

    eng_cc_p_handle
        .calibrate(ADC_CALIBRATION_SAMPLES, Irqs)
        .await;

    // Share the calibration values between all pressure sensors because some of the ADC peripherals
    // are not connected to VREFINT and cannot read it out themselves
    eng_inj_p_handle.vref_calib = eng_cc_p_handle.vref_calib;
    oss_inj_p_handle.vref_calib = eng_cc_p_handle.vref_calib;

    let mut ticker = Ticker::every(Duration::from_millis(
        (1000.0 / ACQ_PRESSURE_FREQ_HZ) as u64,
    ));
    loop {
        let (eng_cc_p, oss_inj_p, fss_inj_p) = join3(
            eng_cc_p_handle.read_pressure(Irqs),
            oss_inj_p_handle.read_pressure(Irqs),
            eng_inj_p_handle.read_pressure(Irqs),
        )
        .await;

        let measurement = EnginePressureMeasurementRaw {
            eng_cc_p,
            oss_inj_p,
            fss_inj_p,
        };

        let filtered = data_publisher.update(measurement);
        info!("Engine CC pressure: {}", filtered.eng_cc_p);
        info!("FSS injector pressure: {}", filtered.fss_inj_p);
        info!("OSS injector pressure: {}", filtered.oss_inj_p);

        // Wait for the next tick
        ticker.next().await;
    }
}
