use crate::drivers::digital_pressure::{
    DigitalPressureDriver, DigitalPressureMeasurementRaw, KELLER_BUS_ERROR_WATCH, TankLevelDriver,
    TankLevelMeasurementRaw,
};
use crate::sensors::{
    ACQ_PRESSURE_FREQUENCY_HZ, ACQ_TANK_LEVEL_FREQUENCY_HZ, OSS_TNK_LVL, OSS_TNK_P1, OSS_TNK_P2,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use embedded_utils::fmt::warn;
use hermes_can::messages::board_status::SensorStatus;
use keller_pressure::KellerSensRS485;

pub(crate) const LEVEL_UPDATE_DURATION: Duration =
    Duration::from_millis((1000.0 / ACQ_TANK_LEVEL_FREQUENCY_HZ) as u64);

#[embassy_executor::task]
pub(crate) async fn keller_acquisition(mut keller_handle: KellerSensRS485<'static>) -> ! {
    let mut pressure_driver = DigitalPressureDriver::new();
    let mut tank_level_driver = TankLevelDriver::new();

    let status_sender = KELLER_BUS_ERROR_WATCH.sender();
    let mut error_count = 0;

    let mut ticker = Ticker::every(Duration::from_millis(
        (1000.0 / ACQ_PRESSURE_FREQUENCY_HZ) as u64,
    ));

    let mut start = Instant::now();
    loop {
        let oss_tnk_p1 = OSS_TNK_P1
            .get_pressure(&mut keller_handle)
            .await
            .unwrap_or_else(|e| {
                warn!("[Keller] Error: {:?}", e);
                error_count += 1;
                f32::NAN
            });

        Timer::after_millis(1).await;

        let oss_tnk_p2 = OSS_TNK_P2
            .get_pressure(&mut keller_handle)
            .await
            .unwrap_or_else(|e| {
                warn!("[Keller] Error: {:?}", e);
                error_count += 1;
                f32::NAN
            });

        Timer::after_millis(1).await;

        pressure_driver.update(DigitalPressureMeasurementRaw {
            oss_tnk_p1,
            oss_tnk_p2,
        });

        if Instant::now() - start >= LEVEL_UPDATE_DURATION {
            let oss_tnk_lvl = OSS_TNK_LVL
                .get_level(&mut keller_handle)
                .await
                .unwrap_or_else(|e| {
                    warn!("[Keller] Error: {:?}", e);
                    error_count += 1;
                    255 // returning Invalid level
                });

            // trace!("[Keller] Tank level: {:?}", oss_tnk_lvl);

            tank_level_driver.update(TankLevelMeasurementRaw { oss_tnk_lvl });

            start = Instant::now();
        }

        if error_count > 0 {
            warn!("[Keller] Pressure sensor errors detected: {}", error_count);
            status_sender.send(SensorStatus::Offline);
            error_count = 0; // Reset error count after sending
        } else {
            status_sender.send(SensorStatus::Online);
        }

        // trace!("[Keller] OSS_TNK_P1: {} barg, OSS_TNK_P2: {} barg", oss_tnk_p1, oss_tnk_p2);

        ticker.next().await;
    }
}
