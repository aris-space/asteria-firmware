//! Readout of the CATS backup recovery flight computer state.

use crate::can_io::OUTPUTS;
use dp_recovery_board::DeploymentState;
use embassy_futures::join::join;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Sender;
use embassy_time::Timer;

/// Reads the CATS IO pins and publishes them to `OUTPUTS`.
///
/// The pins idle high and are pulled low if the CATS gives power to the servos.
/// Upon deployment, it quickly (10ms) resets the power (to the other servo, but we don't care),
/// which we wait for, but handle the fact that we could also have disarmed.
#[embassy_executor::task]
pub async fn cats_task(
    separation: ExtiInput<'static, Async>,
    deployment: ExtiInput<'static, Async>,
) -> ! {
    async fn detect(
        tx: &Sender<'_, CriticalSectionRawMutex, DeploymentState, 1>,
        mut pin: ExtiInput<'static, Async>,
    ) -> ! {
        loop {
            tx.send(DeploymentState::Unarmed);
            pin.wait_for_low().await; // low pin means the CATS is giving pyro power, thus must be armed
            tx.send(DeploymentState::NotYetDeployed);
            pin.wait_for_high().await; // disarmed or deployed, thus figure out which it is:

            Timer::after_millis(20).await; // lets wait if the CATS only pulled it high quickly for 10ms
            if pin.is_high() {
                // we unarmed, thus lets wait for the arming again
                continue;
            }
            tx.send(DeploymentState::Deployed);
            pin.wait_for_high().await; // lets wait until it is now disarmed again
        }
    }

    join(
        detect(&OUTPUTS.cats_separation.sender(), separation),
        detect(&OUTPUTS.cats_deployment.sender(), deployment),
    )
    .await
    .0
}
