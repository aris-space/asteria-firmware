use can_utils::rxtx::TypedCanReceiver;
use defmt::{error, warn};

use super::{ReceivedMessage, THIS_BOARD_ID};

#[embassy_executor::task]
pub async fn task(mut can: TypedCanReceiver) -> ! {
    loop {
        match can.recv::<ReceivedMessage>().await {
            Ok(ReceivedMessage::ResetAll(_)) => {
                warn!("CAN: ResetAll received, resetting");
                cortex_m::peripheral::SCB::sys_reset();
            }
            Ok(ReceivedMessage::ResetSpecific(board)) if board == THIS_BOARD_ID => {
                warn!("CAN: ResetSpecific received, resetting");
                cortex_m::peripheral::SCB::sys_reset();
            }
            Ok(ReceivedMessage::ResetSpecific(_)) => {}
            Err(err) => error!("CAN: RX error: {:?}", err),
        }
    }
}
