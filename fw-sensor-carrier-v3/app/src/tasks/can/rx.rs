use data_core::can::hal::CanDecode as _;
use defmt::{error, warn};
use embassy_stm32::can::BufferedFdCanReceiver;
use embedded_can::Id;

use super::{ReceivedMessage, THIS_BOARD_ID};

#[embassy_executor::task]
pub async fn task(can: BufferedFdCanReceiver) -> ! {
    loop {
        let frame = match can.receive().await {
            Ok(envelope) => envelope.frame,
            Err(err) => {
                error!("CAN: RX error: {:?}", err);
                continue;
            }
        };
        // The hardware filter passes only standard IDs of `ReceivedMessage`.
        let Id::Standard(id) = frame.id() else {
            continue;
        };
        match ReceivedMessage::from_parts(*id, frame.data()) {
            Ok(ReceivedMessage::ResetAll(_)) => {
                warn!("CAN: ResetAll received, resetting");
                cortex_m::peripheral::SCB::sys_reset();
            }
            Ok(ReceivedMessage::ResetSpecific(board)) if board == THIS_BOARD_ID => {
                warn!("CAN: ResetSpecific received, resetting");
                cortex_m::peripheral::SCB::sys_reset();
            }
            Ok(ReceivedMessage::ResetSpecific(_)) => {}
            Err(_) => error!("CAN: undecodable frame with ID {=u16:#x}", id.as_raw()),
        }
    }
}
