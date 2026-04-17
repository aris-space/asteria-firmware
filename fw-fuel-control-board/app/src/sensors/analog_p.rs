use crate::drivers::WATCH;
use crate::drivers::analog_pressure::{
    FSS_TNK_P_WATCH, FILTER_MEAN, FILTER_SIGMA, FILTER_WINDOW, get_filtered_tank_p, raw_to_bar, FuelTankPressureMeasurement
};
use crate::sensors::ACQ_PRESSURE_FREQUENCY_HZ;
use embassy_stm32::adc::{Adc, AnyAdcChannel, Instance};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Ticker};
use embedded_utils::trace;
use filters::GaussianMovingAverage;

#[embassy_executor::task]
pub(crate) async fn analog_pressure_sensor<ADC: Instance + 'static>(
    mut adc: Adc<'static, ADC>,
    mut channel: AnyAdcChannel<'static, ADC>,
    watch_sender: Sender<'static, ThreadModeRawMutex, f32, WATCH>,
) -> ! {
    let mut filter: GaussianMovingAverage<FILTER_WINDOW> =
        GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN);
    let mut ticker =
        Ticker::every(Duration::from_millis(1000 / ACQ_PRESSURE_FREQUENCY_HZ as u64));

    loop {
        let raw = adc.blocking_read(&mut channel);
        let filtered = filter.update(raw_to_bar(raw));
        watch_sender.send(filtered);
        trace!("[Analog] Pressure: {} bar", filtered);
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub(crate) async fn fss_tank_pressure_task<
    ADC_A: Instance + 'static,
    ADC_B: Instance + 'static,
>(
    mut adc_a: Adc<'static, ADC_A>,
    mut ch_a: AnyAdcChannel<'static, ADC_A>,
    mut adc_b: Adc<'static, ADC_B>,
    mut ch_b: AnyAdcChannel<'static, ADC_B>,
) -> ! {
    let p_sender = FSS_TNK_P_WATCH.sender();

    let mut filter_a: GaussianMovingAverage<FILTER_WINDOW> =
        GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN);
    let mut filter_b: GaussianMovingAverage<FILTER_WINDOW> =
        GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN);
    let mut ticker =
        Ticker::every(Duration::from_millis(1000 / ACQ_PRESSURE_FREQUENCY_HZ as u64));

    loop {
        let p1 = filter_a.update(raw_to_bar(adc_a.blocking_read(&mut ch_a)));
        let p2 = filter_b.update(raw_to_bar(adc_b.blocking_read(&mut ch_b)));
        let dpr_pressure = get_filtered_tank_p(p1, p2);

        p_sender.send(FuelTankPressureMeasurement{
            fss_tnk_p1: p1,
            fss_tnk_p2: p2,
            dpr_pressure,
        });

        trace!("[Analog] Tank P1: {} bar, P2: {} bar, DPR: {} bar", p1, p2, dpr_pressure);
        ticker.next().await;
    }
}
