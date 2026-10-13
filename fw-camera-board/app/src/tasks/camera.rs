//! Camera power, following the flight computer's arming state: on as soon as
//! it reports Armed, off once it has reported Safe for [`OFF_DELAY`], so the
//! cameras keep recording for a while after landing or an abort. If the status
//! messages stop, the camera keeps its current state.

use datatypes::status::ArmingState;
use defmt::info;
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, with_timeout};

use crate::signals;

const OFF_DELAY: Duration = Duration::from_secs(5 * 60);

#[embassy_executor::task]
pub async fn task(mut power: Output<'static>) {
    let mut arming = signals::FC_ARMING
        .receiver()
        .expect("camera: FC arming receiver available");

    loop {
        arming
            .changed_and(|state| matches!(state, ArmingState::Armed))
            .await;
        info!("camera: FC armed, powering on");
        power.set_high();

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

        info!("camera: powering off");
        power.set_low();
    }
}
