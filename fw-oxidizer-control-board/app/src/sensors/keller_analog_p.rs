use crate::Irqs;
use crate::drivers::analog_pressure::{OxidizerPressureDriver, OxidizerPressureMeasurementRaw};
use crate::sensors::{
    ACQ_PRESSURE_FREQ_HZ, ADC_CALIBRATION_SAMPLES, OXIDIZER_TANK_DIFFERENTIAL_PRESSURE_RANGE,
    OXIDIZER_TANK_PRESSURE_1_RANGE, OXIDIZER_TANK_PRESSURE_2_RANGE,
};
use analog_pressure::ADCPressure;
use analog_pressure::pressures::TrafagPSens;
use embassy_futures::join::join3;
use embassy_stm32::Peri;
use embassy_stm32::adc::AdcChannel;
use embassy_stm32::peripherals::{ADC1, ADC2, ADC3, DMA1_CH3, DMA1_CH4, DMA2_CH3, PB13, PC0, PC1};
use embassy_time::{Duration, Ticker};
use embedded_utils::fmt::info;

pub struct OxidizerPressureHandles {
    pub oxidizer_tank_pressure_1_adc: Peri<'static, ADC1>,
    pub oxidizer_tank_pressure_1_dma: Peri<'static, DMA1_CH4>,
    pub oxidizer_tank_pressure_1_pin: Peri<'static, PC0>,
    pub oxidizer_tank_pressure_2_adc: Peri<'static, ADC2>,
    pub oxidizer_tank_pressure_2_dma: Peri<'static, DMA2_CH3>,
    pub oxidizer_tank_pressure_2_pin: Peri<'static, PC1>,
    pub oxidizer_tank_differential_pressure_adc: Peri<'static, ADC3>,
    pub oxidizer_tank_differential_pressure_dma: Peri<'static, DMA1_CH3>,
    pub oxidizer_tank_differential_pressure_pin: Peri<'static, PB13>,
}

#[embassy_executor::task]
pub async fn oxidizer_pressure_acquisition(pressure_handles: OxidizerPressureHandles) {
    let mut data_publisher = OxidizerPressureDriver::new();

    let mut oxidizer_tank_pressure_1_pin = pressure_handles.oxidizer_tank_pressure_1_pin;
    let mut oxidizer_tank_pressure_2_pin = pressure_handles.oxidizer_tank_pressure_2_pin;
    let mut oxidizer_tank_differential_pressure_pin =
        pressure_handles.oxidizer_tank_differential_pressure_pin;

    let mut oxidizer_tank_pressure_1_handle = ADCPressure::new(
        pressure_handles.oxidizer_tank_pressure_1_adc,
        pressure_handles.oxidizer_tank_pressure_1_dma,
        TrafagPSens {
            pin: oxidizer_tank_pressure_1_pin.degrade_adc(),
            si_range: OXIDIZER_TANK_PRESSURE_1_RANGE,
        },
    )
    .await;

    let mut oxidizer_tank_pressure_2_handle = ADCPressure::new(
        pressure_handles.oxidizer_tank_pressure_2_adc,
        pressure_handles.oxidizer_tank_pressure_2_dma,
        TrafagPSens {
            pin: oxidizer_tank_pressure_2_pin.degrade_adc(),
            si_range: OXIDIZER_TANK_PRESSURE_2_RANGE,
        },
    )
    .await;

    let mut oxidizer_tank_differential_pressure_handle = ADCPressure::new(
        pressure_handles.oxidizer_tank_differential_pressure_adc,
        pressure_handles.oxidizer_tank_differential_pressure_dma,
        TrafagPSens {
            pin: oxidizer_tank_differential_pressure_pin.degrade_adc(),
            si_range: OXIDIZER_TANK_DIFFERENTIAL_PRESSURE_RANGE,
        },
    )
    .await;

    oxidizer_tank_pressure_1_handle
        .calibrate(ADC_CALIBRATION_SAMPLES, Irqs)
        .await;
    // Share the calibration values between all pressure sensors because some ADC peripherals
    // are not connected to VREFINT and cannot read it out themselves.
    oxidizer_tank_pressure_2_handle.vref_calib = oxidizer_tank_pressure_1_handle.vref_calib;
    oxidizer_tank_differential_pressure_handle.vref_calib =
        oxidizer_tank_pressure_1_handle.vref_calib;

    let mut ticker = Ticker::every(Duration::from_micros(
        (1_000_000.0 / ACQ_PRESSURE_FREQ_HZ + 0.5) as u64,
    ));
    loop {
        let (
            oxidizer_tank_pressure_1,
            oxidizer_tank_pressure_2,
            oxidizer_tank_differential_pressure,
        ) = join3(
            oxidizer_tank_pressure_1_handle.read_pressure(Irqs),
            oxidizer_tank_pressure_2_handle.read_pressure(Irqs),
            oxidizer_tank_differential_pressure_handle.read_pressure(Irqs),
        )
        .await;

        let measurement = OxidizerPressureMeasurementRaw {
            oxidizer_tank_pressure_1,
            oxidizer_tank_pressure_2,
            oxidizer_tank_differential_pressure,
        };

        info!(
            "Oxidizer tank pressure 1: {}, Oxidizer tank pressure 2: {}, Oxidizer tank differential pressure: {}",
            measurement.oxidizer_tank_pressure_1,
            measurement.oxidizer_tank_pressure_2,
            measurement.oxidizer_tank_differential_pressure
        );

        data_publisher.update(measurement);
        ticker.next().await;
    }
}
