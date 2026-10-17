use can_utils::rxtx::RxError;
use defmt::{error, warn};
use embassy_stm32::can::BufferedFdCanReceiver;

use crate::tasks::can::{ReceivedMessage, THIS_BOARD_ID, utils::BufferedCanReceive as _};

#[embassy_executor::task]
pub async fn task(mut can: BufferedFdCanReceiver) -> ! {
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
            Err(RxError::Bug) => {}
            Err(err) => error!("CAN: RX error: {:?}", err),
        }
    }
}
