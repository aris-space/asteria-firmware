use crate::drivers::digital_pressure::{
    DigitalPressureDriver, DigitalPressureMeasurementRaw, KELLER_BUS_ERROR_WATCH,
};
use crate::sensors::{ACQ_PRESSURE_FREQUENCY_HZ, FSS_TNK_P1, FSS_TNK_P2, PRZ_MNL_P};
use embassy_time::{Duration, Ticker, Timer};
use embedded_utils::error;
use embedded_utils::fmt::warn;
use hermes_can::messages::board_status::SensorStatus;
use keller_pressure::KellerSensRS485;

#[embassy_executor::task]
pub(crate) async fn keller_acquisition(mut keller_handle: KellerSensRS485<'static>) -> ! {
    let mut pressure_driver = DigitalPressureDriver::new();

    let status_sender = KELLER_BUS_ERROR_WATCH.sender();
    let mut error_count = 0;

    let mut ticker = Ticker::every(Duration::from_millis(
        1000 / ACQ_PRESSURE_FREQUENCY_HZ as u64,
    ));

    loop {
        let prz_mnl_p = PRZ_MNL_P
            .get_pressure(&mut keller_handle)
            .await
            .unwrap_or_else(|e| {
                error!("[Keller] Error: {:?}", e);
                error_count += 1;
                f32::NAN
            });
        Timer::after_millis(1).await;

        let fss_tnk_p1 = FSS_TNK_P1
            .get_pressure(&mut keller_handle)
            .await
            .unwrap_or_else(|e| {
                error!("[Keller] Error: {:?}", e);
                error_count += 1;
                f32::NAN
            });
        Timer::after_millis(1).await;

        let fss_tnk_p2 = FSS_TNK_P2
            .get_pressure(&mut keller_handle)
            .await
            .unwrap_or_else(|e| {
                error!("[Keller] Error: {:?}", e);
                error_count += 1;
                f32::NAN
            });
        Timer::after_millis(1).await;

        pressure_driver.update(DigitalPressureMeasurementRaw {
            prz_mnl_p,
            fss_tnk_p1,
            fss_tnk_p2,
        });

        /*trace!(
            "[Keller] PRZ_MNL_P: {} barg, FUE_TNK_P1: {} barg, FUE_TNK_P2: {} barg",
            prz_mnl_p,
            fss_tnk_p1,
            fss_tnk_p2
        );*/
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
