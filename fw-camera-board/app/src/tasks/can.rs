use can_utils::rxtx::{RxError, TypedCanReceive as _};
use data_core::can::hal::CanDecode as _;
use defmt::{error, warn};
use embassy_stm32::can::CanRx;
use embedded_can::StandardId;

use crate::signals;

// TODO: no ResetSpecific yet, there is no BoardId for this board
data_core::can::sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        FlightComputerStatus(dp_flight_computer::Message::Status),
    }
}

pub const RECEIVED_IDS: &[StandardId] = ReceivedMessage::SUPPORTED_IDS;

#[embassy_executor::task]
pub async fn rx(mut can: CanRx<'static>) -> ! {
    loop {
        match can.recv::<ReceivedMessage>().await {
            Ok(ReceivedMessage::ResetAll(_)) => {
                warn!("CAN: ResetAll received, resetting");
                cortex_m::peripheral::SCB::sys_reset();
            }
            Ok(ReceivedMessage::FlightComputerStatus(status)) => {
                signals::FC_ARMING.sender().send(status.is_software_armed);
            }
            Err(RxError::Bug) => {}
            Err(err) => error!("CAN: RX error: {:?}", err),
        }
    }
}
