use crate::drivers::analog_pressure::{
    FILTER_MEAN, FILTER_SIGMA, FILTER_WINDOW, FuelTankPressureMeasurement, get_filtered_tank_p,
    raw_to_bar,
};
use crate::globals::STATE;
use crate::sensors::ACQ_PRESSURE_FREQUENCY_HZ;
use datatypes::units::BarG;
use embassy_stm32::Peri;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::peripherals::{ADC1, ADC2, ADC3, PB13, PC0, PC1};
use embassy_time::{Duration, Ticker};
use embedded_utils::trace;
use filters::GaussianMovingAverage;

#[embassy_executor::task]
pub async fn analog_pressure_sensor(
    mut adc: Adc<'static, ADC1>,
    mut channel: Peri<'static, PC0>,
) -> ! {
    let p_sender = STATE.pressurization_pressure.sender();

    let mut filter: GaussianMovingAverage<FILTER_WINDOW> =
        GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN);
    let mut ticker = Ticker::every(Duration::from_millis(
        1000 / ACQ_PRESSURE_FREQUENCY_HZ as u64,
    ));

    loop {
        let raw = adc.blocking_read(&mut channel, SampleTime::CYCLES2_5);
        let filtered = filter.update(raw_to_bar(raw));
        p_sender.send(BarG(filtered));
        trace!("[Analog] Pressure: {} bar", filtered);
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn fss_tank_pressure_task(
    mut adc_a: Adc<'static, ADC2>,
    mut ch_a: Peri<'static, PC1>,
    mut adc_b: Adc<'static, ADC3>,
    mut ch_b: Peri<'static, PB13>,
) -> ! {
    let p_sender = STATE.fuel_tank_pressure.sender();

    let mut filter_a: GaussianMovingAverage<FILTER_WINDOW> =
        GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN);
    let mut filter_b: GaussianMovingAverage<FILTER_WINDOW> =
        GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN);
    let mut ticker = Ticker::every(Duration::from_millis(
        1000 / ACQ_PRESSURE_FREQUENCY_HZ as u64,
    ));

    loop {
        let p1 = filter_a.update(raw_to_bar(
            adc_a.blocking_read(&mut ch_a, SampleTime::CYCLES2_5),
        ));
        let p2 = filter_b.update(raw_to_bar(
            adc_b.blocking_read(&mut ch_b, SampleTime::CYCLES2_5),
        ));
        let dpr_pressure = get_filtered_tank_p(p1, p2);

        p_sender.send(FuelTankPressureMeasurement {
            fuel_tank_pressure_1: BarG(p1),
            fuel_tank_pressure_2: BarG(p2),
            dpr_pressure: BarG(dpr_pressure),
        });

        trace!(
            "[Analog] Tank P1: {} bar, P2: {} bar, DPR: {} bar",
            p1, p2, dpr_pressure
        );
        ticker.next().await;
    }
}
