use crate::drivers::digital_pressure::{
    DigitalPressureMeasurementRaw, DigitalTemperatureMeasurementRaw, KELLER_BUS_ERROR_WATCH,
    KellerDriver,
};
use crate::sensors::{ACQ_PRESSURE_FREQ_HZ, ENG_CC_P, FUE_INJ_P, OXD_INJ_P};
use embassy_time::{Duration, Ticker, Timer};
use embedded_utils::fmt::warn;
use embedded_utils::{error, trace};
use hermes_can::messages::board_status::SensorStatus;
use keller_pressure::KellerSensRS485;

#[embassy_executor::task]
pub(crate) async fn keller_acquisition(mut keller_handle: KellerSensRS485<'static>) -> ! {
    let mut pressure_driver = KellerDriver::new();

    let status_sender = KELLER_BUS_ERROR_WATCH.sender();
    let mut error_count = 0;

    let mut ticker = Ticker::every(Duration::from_millis(1000 / ACQ_PRESSURE_FREQ_HZ as u64));

    loop {
        let (eng_cc_p, eng_cc_t) = ENG_CC_P
            .get_pressure_temperature(&mut keller_handle)
            .await
            .unwrap_or_else(|e| {
                error!("[Keller] Error: {:?}", e);
                error_count += 1;
                (f32::NAN, f32::INFINITY)
            });

        Timer::after_millis(1).await;

        let (fue_inj_p, fue_inj_t) = FUE_INJ_P
            .get_pressure_temperature(&mut keller_handle)
            .await
            .unwrap_or_else(|e| {
                error!("[Keller] Error: {:?}", e);
                error_count += 1;
                (f32::NAN, f32::INFINITY)
            });

        Timer::after_millis(1).await;

        let (oxd_inj_p, oxd_inj_t) = OXD_INJ_P
            .get_pressure_temperature(&mut keller_handle)
            .await
            .unwrap_or_else(|e| {
                error!("[Keller] Error: {:?}", e);
                error_count += 1;
                (f32::NAN, f32::INFINITY)
            });

        Timer::after_millis(1).await;

        pressure_driver.update(
            DigitalPressureMeasurementRaw {
                eng_cc_p,
                fue_inj_p,
                oxd_inj_p,
            },
            DigitalTemperatureMeasurementRaw {
                eng_cc_t,
                fue_inj_t,
                oxd_inj_t,
            },
        );

        trace!(
            "[Keller] (ENG_P: {} barg, ENG_T: {} °C), (FUE_P: {} barg, FUE_T: {} °C), (OXD_P: {} barg, OXD_T: {} °C)",
            eng_cc_p, eng_cc_t, fue_inj_p, fue_inj_t, oxd_inj_p, oxd_inj_t
        );

        if error_count > 0 {
            warn!("[Keller] Pressure sensor errors detected: {}", error_count);
            status_sender.send(SensorStatus::Offline);
            error_count = 0; // Reset error count after sending
        } else {
            status_sender.send(SensorStatus::Online);
        }
        ticker.next().await;
    }
}
