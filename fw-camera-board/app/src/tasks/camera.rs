//! Powers the camera while the FC is armed and starts/stops recording over the
//! RunCam UART, so the file is closed before power is cut. It stays on for
//! [`OFF_DELAY`] after Safe because booting it again takes seconds.

use datatypes::status::ArmingState;
use defmt::{info, warn};
use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer, with_timeout};

use crate::runcam::{self, RunCam};
use crate::signals;

const OFF_DELAY: Duration = Duration::from_secs(5 * 60);
/// Spreads the cameras' inrush currents.
// TODO: tune on hw, 0 disables the stagger
const STAGGER_STEP: Duration = Duration::from_millis(0);

/// How recording is stopped before power is cut.
enum Stop {
    Command,
    PowerButton,
    Unsupported,
}

#[embassy_executor::task]
pub async fn task(mut power: Output<'static>, mut camera: RunCam, address: u8) {
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

        let features = camera.wait_until_booted(runcam::BOOT_TIMEOUT).await;
        let stop = match features {
            Some(features) if features & runcam::FEATURE_START_RECORDING != 0 => {
                info!("camera: starting recording");
                camera.start_recording().await;
                if features & runcam::FEATURE_STOP_RECORDING != 0 {
                    Stop::Command
                } else if features & runcam::FEATURE_SIMULATE_POWER_BUTTON != 0 {
                    // The Split 4 V3 reports start but not stop recording.
                    Stop::PowerButton
                } else {
                    Stop::Unsupported
                }
            }
            Some(_) => {
                warn!("camera: no start recording support, relying on auto-record");
                Stop::Unsupported
            }
            None => {
                warn!(
                    "camera: no UART reply within {} s, relying on auto-record",
                    runcam::BOOT_TIMEOUT.as_secs()
                );
                Stop::Unsupported
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

        match stop {
            Stop::Command => {
                info!("camera: stopping recording");
                camera.stop_recording().await;
                Timer::after(runcam::FILE_CLOSE_DELAY).await;
            }
            Stop::PowerButton => {
                info!("camera: stopping recording with a power button press");
                camera.press_power_button().await;
                Timer::after(runcam::FILE_CLOSE_DELAY).await;
            }
            Stop::Unsupported => {
                warn!("camera: cannot stop recording over UART, file may be lost");
            }
        }

        info!("camera: powering off");
        power.set_low();
    }
}
