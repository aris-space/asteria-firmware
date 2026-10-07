//! Readout of the CATS backup recovery flight computer state.

use crate::actuator_control::ARMING_STATE;
use crate::can_io::OUTPUTS;
use datatypes::status::ArmingState;
use dp_recovery_board::DeploymentState;
use embassy_stm32::gpio::Input;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Sender;
use embassy_time::Timer;

/// Poll interval for the CATS outputs (20 Hz)
const CATS_POLL_MS: u64 = 50;

/// Reads the CATS IO pins and publishes them to `OUTPUTS`.
///
/// The pins idle high and are pulled low once CATS has triggered the action. Until the board is armed
/// the state is reported as [`DeploymentState::Unarmed`].
#[embassy_executor::task]
pub async fn cats_task(separation: Input<'static>, deployment: Input<'static>) {
    /// Publish the state of one CATS pin, only notifying receivers (the CAN broadcast) on a change.
    fn publish(
        tx: &Sender<'_, CriticalSectionRawMutex, DeploymentState, 1>,
        armed: Option<ArmingState>,
        pin: &Input<'static>,
    ) {
        let new = match (armed, pin.is_low()) {
            (Some(ArmingState::Armed), false) => DeploymentState::NotYetDeployed,
            (Some(ArmingState::Armed), true) => DeploymentState::Deployed,
            (_, _) => DeploymentState::Unarmed,
        };
        tx.send_if_modified(|current| {
            let changed = *current != Some(new);
            *current = Some(new);
            changed
        });
    }

    let mut arming_rx = ARMING_STATE.receiver().unwrap();
    let separation_tx = OUTPUTS.cats_separation.sender();
    let deployment_tx = OUTPUTS.cats_deployment.sender();

    loop {
        let armed = arming_rx.try_get();
        publish(&separation_tx, armed, &separation);
        publish(&deployment_tx, armed, &deployment);
        Timer::after_millis(CATS_POLL_MS).await;
    }
}
