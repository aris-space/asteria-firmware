//! CAN bus: the RX task handles resets, the TX task sends every
//! `dp-sensor-carrier` message.

use data_core::can::hal::CanDecode as _;
use datatypes::status::BoardId;
use embedded_can::StandardId;

pub mod rx;
pub mod tx;

pub const THIS_BOARD_ID: BoardId = BoardId::SensorCarrier;

// The only messages the Sensor Carrier receives.
data_core::can::sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
    }
}

/// The IDs of [`ReceivedMessage`], the only ones the hardware filter passes.
pub const RECEIVED_IDS: &[StandardId] = ReceivedMessage::SUPPORTED_IDS;
