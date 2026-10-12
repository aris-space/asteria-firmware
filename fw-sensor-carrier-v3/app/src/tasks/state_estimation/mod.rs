//! Feeds the calibrated sensor streams into SEF-light and publishes its estimate.
//!
//! SEF-light fuses every reading [`config::FUSION_DELAY_US`] behind the newest IMU
//! sample, in time order, and carries the state forward to it with the IMU.
//! Positions are NED metres from [`position_reference`].

mod config;
mod processor;

use asteria_sef_core::GeodeticReference;
use defmt::{Debug2Format, warn};
use embassy_futures::select::{Either4, select4};
use embassy_sync::pubsub::WaitResult;
use embassy_time::Instant;

use processor::Processor;

use crate::signals;

/// NED reference at the launch-site coordinates and zero MSL height.
pub fn position_reference() -> GeodeticReference {
    GeodeticReference::new(
        config::LAUNCH_SITE_LATITUDE_DEG,
        config::LAUNCH_SITE_LONGITUDE_DEG,
        0.0,
    )
    .expect("position reference must be valid")
}

/// Whether GNSS has referenced the height to MSL, rather than only the barometers.
pub fn height_msl_referenced(height_std_m: f32) -> bool {
    height_std_m < config::MSL_REFERENCED_HEIGHT_STD_M
}

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut imu = signals::IMU_CHANNEL
        .subscriber()
        .expect("SEF: subscriber slot");
    let mut mag = signals::MAG_CHANNEL
        .subscriber()
        .expect("SEF: subscriber slot");
    let mut gnss = signals::GNSS_CHANNEL
        .subscriber()
        .expect("SEF: subscriber slot");
    let mut baro = signals::BARO_CHANNEL
        .subscriber()
        .expect("SEF: subscriber slot");
    let estimator = config::new_estimator().expect("SEF-light configuration must be valid");
    let mut processor = Processor::new(estimator);
    let mut last_warning: Option<Instant> = None;

    loop {
        let result = match select4(
            imu.next_message(),
            mag.next_message(),
            gnss.next_message(),
            baro.next_message(),
        )
        .await
        {
            Either4::First(WaitResult::Message(reading)) => processor.update_imu(reading.cal),
            Either4::Second(WaitResult::Message(reading)) => processor.update_mag(reading.cal),
            Either4::Third(WaitResult::Message(reading)) => processor.update_gnss(reading.cal),
            Either4::Fourth(WaitResult::Message(reading)) => processor.update_baro(reading.cal),
            Either4::First(WaitResult::Lagged(lost))
            | Either4::Second(WaitResult::Lagged(lost))
            | Either4::Third(WaitResult::Lagged(lost))
            | Either4::Fourth(WaitResult::Lagged(lost)) => {
                warn!("SEF: fell behind and lost {} readings", lost);
                continue;
            }
        };

        if let Err(error) = result {
            let now = Instant::now();
            if last_warning
                .is_none_or(|last| now.saturating_duration_since(last) >= config::WARNING_PERIOD)
            {
                warn!("SEF: update failed: {:?}", Debug2Format(&error));
                last_warning = Some(now);
            }
        }
        processor.publish();
    }
}
