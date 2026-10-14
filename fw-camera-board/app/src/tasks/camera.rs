//! The camera records whenever powered. It stays on for [`OFF_DELAY`] after Safe
//! because booting it again takes seconds.

use datatypes::status::ArmingState;
use defmt::info;
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer, with_timeout};

use crate::signals;

const OFF_DELAY: Duration = Duration::from_secs(5 * 60);
/// Spreads the cameras' inrush currents.
const STAGGER_STEP: Duration = Duration::from_millis(200);

#[embassy_executor::task]
pub async fn task(mut power: Output<'static>, address: u8) {
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
        power.set_high();

        // TODO: untested on hw
        // if camera.wait_until_booted(runcam::BOOT_TIMEOUT).await {
        //     camera.start_recording().await;
        // }

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

        // TODO: untested on hw
        // camera.stop_recording().await;
        // Timer::after(runcam::FILE_CLOSE_DELAY).await;

        info!("camera: powering off");
        power.set_low();
    }
}
