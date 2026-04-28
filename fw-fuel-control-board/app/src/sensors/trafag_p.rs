use crate::Irqs;
use crate::drivers::pressure::{FuelPressureDriver, FuelPressureMeasurementRaw};
use crate::sensors::{
    ACQ_PRESSURE_FREQ_HZ, ADC_CALIBRATION_SAMPLES, FUEL_TANK_PRESSURE_1_RANGE,
    FUEL_TANK_PRESSURE_2_RANGE, PRESSURIZATION_PRESSURE_RANGE,
};
use embassy_stm32::Peri;
use embassy_stm32::adc::AdcChannel;
use embassy_stm32::peripherals::{ADC1, ADC2, ADC3, DMA1_CH3, DMA1_CH4, DMA2_CH3, PB13, PC0, PC1};
use embassy_time::{Duration, Ticker};
use trafag_pressure::ADCPressure;
use trafag_pressure::pressures::TrafagPSens;

pub struct FuelPressureHandles {
    pub pressurization_pressure_adc: Peri<'static, ADC1>,
    pub pressurization_pressure_dma: Peri<'static, DMA1_CH4>,
    pub pressurization_pressure_pin: Peri<'static, PC0>,
    pub fuel_tank_pressure_1_adc: Peri<'static, ADC2>,
    pub fuel_tank_pressure_1_dma: Peri<'static, DMA2_CH3>,
    pub fuel_tank_pressure_1_pin: Peri<'static, PC1>,
    pub fuel_tank_pressure_2_adc: Peri<'static, ADC3>,
    pub fuel_tank_pressure_2_dma: Peri<'static, DMA1_CH3>,
    pub fuel_tank_pressure_2_pin: Peri<'static, PB13>,
}

#[embassy_executor::task]
pub async fn fuel_pressure_acquisition(pressure_handles: FuelPressureHandles) {
    let mut data_publisher = FuelPressureDriver::new();

    let mut pressurization_pressure_pin = pressure_handles.pressurization_pressure_pin;
    let mut fuel_tank_pressure_1_pin = pressure_handles.fuel_tank_pressure_1_pin;
    let mut fuel_tank_pressure_2_pin = pressure_handles.fuel_tank_pressure_2_pin;

    let mut pressurization_pressure_handle = ADCPressure::new(
        pressure_handles.pressurization_pressure_adc,
        pressure_handles.pressurization_pressure_dma,
        TrafagPSens {
            pin: pressurization_pressure_pin.degrade_adc(),
            si_range: PRESSURIZATION_PRESSURE_RANGE,
        },
    )
    .await;

    let mut fuel_tank_pressure_1_handle = ADCPressure::new(
        pressure_handles.fuel_tank_pressure_1_adc,
        pressure_handles.fuel_tank_pressure_1_dma,
        TrafagPSens {
            pin: fuel_tank_pressure_1_pin.degrade_adc(),
            si_range: FUEL_TANK_PRESSURE_1_RANGE,
        },
    )
    .await;

    let mut fuel_tank_pressure_2_handle = ADCPressure::new(
        pressure_handles.fuel_tank_pressure_2_adc,
        pressure_handles.fuel_tank_pressure_2_dma,
        TrafagPSens {
            pin: fuel_tank_pressure_2_pin.degrade_adc(),
            si_range: FUEL_TANK_PRESSURE_2_RANGE,
        },
    )
    .await;

    pressurization_pressure_handle
        .calibrate(ADC_CALIBRATION_SAMPLES, Irqs)
        .await;

    // Share the calibration values between all pressure sensors because some of the ADC peripherals
    // are not connected to VREFINT and cannot read it out themselves.
    fuel_tank_pressure_1_handle.vref_calib = pressurization_pressure_handle.vref_calib;
    fuel_tank_pressure_2_handle.vref_calib = pressurization_pressure_handle.vref_calib;

    let mut ticker = Ticker::every(Duration::from_millis(
        (1000.0 / ACQ_PRESSURE_FREQ_HZ) as u64,
    ));
    loop {
        let measurement = FuelPressureMeasurementRaw {
            pressurization_pressure: pressurization_pressure_handle.read_pressure(Irqs).await,
            fuel_tank_pressure_1: fuel_tank_pressure_1_handle.read_pressure(Irqs).await,
            fuel_tank_pressure_2: fuel_tank_pressure_2_handle.read_pressure(Irqs).await,
        };

        data_publisher.update(measurement);

        ticker.next().await;
    }
}
