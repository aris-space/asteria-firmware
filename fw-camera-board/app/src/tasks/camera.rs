//! The camera records whenever powered. It stays on for [`OFF_DELAY`] after Safe
//! because booting it again takes seconds.

use datatypes::status::ArmingState;
use defmt::{info, warn};
use embassy_time::{Duration, Timer, with_timeout};

use crate::runcam::{Features, RunCam};
use crate::signals;

const OFF_DELAY: Duration = Duration::from_secs(5 * 60);
/// Spreads the cameras' inrush currents.
// TODO: tune on hw, 0 disables the stagger
const STAGGER_STEP: Duration = Duration::from_millis(0);

#[embassy_executor::task]
pub async fn task(mut runcam: RunCam, address: u8) {
    let mut arming = signals::FC_ARMING
        .receiver()
        .expect("camera: FC arming receiver available");

    loop {
        arming
            .changed_and(|state| matches!(state, ArmingState::Armed))
            .await;
        let stagger = STAGGER_STEP * u32::from(address);
        info!(
            "camera: FC armed, waiting {} ms to power on",
            stagger.as_millis()
        );
        Timer::after(stagger).await;
        info!("camera: powering on");
        let mut camera = runcam.power_on();
        let features = match camera.wait_until_booted().await {
            Ok(features) => {
                info!("camera: booted, features {=u16:#06x}", features.bits());
                if let Err(e) = camera.start_recording().await {
                    warn!(
                        "camera: start recording failed, relying on auto-record: {}",
                        e
                    );
                }
                features
            }
            Err(e) => {
                warn!("camera: no reply after boot, relying on auto-record: {}", e);
                Features::empty()
            }
        };

        loop {
            arming
                .changed_and(|state| matches!(state, ArmingState::Safe))
                .await;
            info!(
                "camera: FC safe, powering off in {} s unless re-armed",
                OFF_DELAY.as_secs()
            );

            let rearmed = with_timeout(
                OFF_DELAY,
                arming.changed_and(|state| matches!(state, ArmingState::Armed)),
            )
            .await;
            if rearmed.is_err() {
                break;
            }
            info!("camera: FC re-armed, staying on");
        }

        match camera.stop_recording(features).await {
            Ok(()) => info!("camera: recording stopped"),
            Err(e) => warn!("camera: stop recording failed, file may be lost: {}", e),
        }

        info!("camera: powering off");
        camera.power_off();
    }
}
