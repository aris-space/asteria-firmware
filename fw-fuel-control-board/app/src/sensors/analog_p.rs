use crate::drivers::WATCH;
use crate::drivers::analog_pressure::{FILTER_MEAN, FILTER_SIGMA, FILTER_WINDOW, raw_to_bar};
use crate::sensors::ACQ_PRESSURE_FREQUENCY_HZ;
use embassy_stm32::adc::{Adc, AnyAdcChannel, Instance};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Ticker};
use embedded_utils::trace;
use filters::GaussianMovingAverage;

/// One acquisition task per analog pressure sensor.
///
/// Spawn once for each sensor, passing the corresponding ADC peripheral,
/// its channel, and the watch sender for that sensor.  Example:
///
/// ```ignore
/// spawner.spawn(analog_pressure_sensor(adc1, prz_mnl_p_ch, PRZ_MNL_P_WATCH.sender())).unwrap();
/// spawner.spawn(analog_pressure_sensor(adc2, fss_tnk_p1_ch, FSS_TNK_P1_WATCH.sender())).unwrap();
/// spawner.spawn(analog_pressure_sensor(adc3, fss_tnk_p2_ch, FSS_TNK_P2_WATCH.sender())).unwrap();
/// ```
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
