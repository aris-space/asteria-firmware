use crate::drivers::pressure::{AnalogPressureDriver, AnalogPressureMeasurementRaw};
use crate::sensors::{ACQ_PRESSURE_FREQ_HZ, ADC_CALIBRATION_SAMPLES};
use embassy_stm32::peripherals::{ADC1, ADC2, ADC3, DMA1_CH3, DMA1_CH4, DMA2_CH3};
use embassy_time::{Duration, Ticker};
use trafag_pressure::ADCPressure;

pub struct EnginePressureHandles {
    pub eng_cc_p_handle: ADCPressure<'static, ADC3, DMA1_CH3>,
    pub fss_inj_p_handle: ADCPressure<'static, ADC1, DMA1_CH4>,
    pub oss_inj_p_handle: ADCPressure<'static, ADC2, DMA2_CH3>,
}

#[embassy_executor::task]
pub async fn engine_pressure_acquisition(pressure_handles: EnginePressureHandles) {
    let mut data_publisher = AnalogPressureDriver::new();

    let mut eng_cc_p_handle = pressure_handles.eng_cc_p_handle;
    let mut oss_inj_p_handle = pressure_handles.oss_inj_p_handle;
    let mut fss_inj_p_handle = pressure_handles.fss_inj_p_handle;
    eng_cc_p_handle.calibrate(ADC_CALIBRATION_SAMPLES).await;

    // Share the calibration values between all pressure sensors because some of the ADC peripherals
    // are not connected to VREFINT and cannot read it out themselves
    oss_inj_p_handle.vref_calib = eng_cc_p_handle.vref_calib;
    fss_inj_p_handle.vref_calib = eng_cc_p_handle.vref_calib;

    let mut ticker = Ticker::every(Duration::from_millis(
        (1000.0 / ACQ_PRESSURE_FREQ_HZ) as u64,
    ));
    loop {
        let measurement = AnalogPressureMeasurementRaw {
            eng_cc_p: eng_cc_p_handle.read_pressure().await,
            oss_inj_p: oss_inj_p_handle.read_pressure().await,
            fss_inj_p: fss_inj_p_handle.read_pressure().await,
        };

        data_publisher.update(measurement);

        // Wait for the next tick
        ticker.next().await;
    }
}
