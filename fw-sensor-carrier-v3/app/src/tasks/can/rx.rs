use can_utils::rxtx::TypedCanReceive as _;
use defmt::{error, warn};
use embassy_stm32::can::CanRx;

use super::{ReceivedMessage, THIS_BOARD_ID};

#[embassy_executor::task]
pub async fn task(mut can_rx: CanRx<'static>) -> ! {
    loop {
        match can_rx.recv::<ReceivedMessage>().await {
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
