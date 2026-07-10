//! CAN bus: RX task handles reset/control frames, TX tasks subscribe to
//! derived signals and emit hermes-can frames at rate-limited cadences.

use embassy_stm32::can::enums::BusError;
use embassy_stm32::can::frame::{self, FdFrame, Header};
use embassy_stm32::can::{Can, CanRx, CanTx};
use embassy_time::TimeoutError;
use embedded_can::Id;
use hermes_can::messages::Message;
use hermes_can::{CanDecodeError, CanEncodeError, CanMessage, next_valid_length};

pub mod rx;
pub mod tx;

pub use rx::rx_task;
pub use tx::spawn_tx_tasks;

pub const THIS_BOARD_ID: hermes_can::messages::BoardId =
    hermes_can::messages::BoardId::SensorCarrier;

#[allow(dead_code)]
#[derive(Debug, thiserror::Error, defmt::Format)]
pub enum CanError {
    #[error("CAN bus error")]
    Bus(BusError),

    #[error("CAN timeout")]
    Timeout(TimeoutError),

    #[error("Encoding CAN message failed")]
    Encode(#[from] CanEncodeError),

    #[error("Decoding CAN message failed")]
    Decode(#[from] CanDecodeError),

    #[error("Invalid frame or Id sent/received")]
    Other,
}

pub trait CanTransmitter {
    async fn transmit<M: CanMessage>(&mut self, msg: M) -> Result<(), CanError>;
}

pub trait CanReceiver {
    async fn recv(&mut self) -> Result<(Message, frame::Timestamp), CanError>;
}

macro_rules! impl_can_transmitter {
    ($ty:ty) => {
        impl CanTransmitter for $ty {
            async fn transmit<M: CanMessage>(&mut self, msg: M) -> Result<(), CanError> {
                let mut buf = [0u8; 64];
                let (id, len) = msg.try_write_into(&mut buf)?;
                let dlc = next_valid_length(len).ok_or(CanError::Other)?;
                let payload = &buf[..dlc];
                let frame = FdFrame::new(Header::new(id.into(), dlc as u8, false), payload)
                    .map_err(|_| CanError::Other)?;
                if let Some(pushed) = self.write_fd(&frame).await {
                    defmt::warn!("CAN dropped frame: {:?}", pushed);
                }
                Ok(())
            }
        }
    };
}

macro_rules! impl_can_receiver {
    ($ty:ty) => {
        impl CanReceiver for $ty {
            async fn recv(&mut self) -> Result<(Message, frame::Timestamp), CanError> {
                let envelope = self.read_fd().await.map_err(CanError::Bus)?;
                let frame = envelope.frame;
                let id = match frame.id() {
                    Id::Standard(id) => id,
                    Id::Extended(_) => return Err(CanError::Other),
                };
                let msg = Message::try_from_parts(*id, frame.data())?;
                Ok((msg, envelope.ts))
            }
        }
    };
}

impl_can_receiver!(Can<'_>);
impl_can_transmitter!(Can<'_>);
impl_can_receiver!(CanRx<'_>);
impl_can_transmitter!(CanTx<'_>);
