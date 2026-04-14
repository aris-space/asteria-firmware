//! This modules defines and implements traits to received typed CAN messages.

use can_hal::{CanDecode, CanEncode};
use embassy_stm32::can::{
    CanRx, CanTx,
    enums::BusError,
    frame::{FdFrame, Header},
};
use embedded_can::Id;
use embedded_utils::fmt::warn;

/// Trait for types that can transmit CAN messages asynchronously.
pub trait TypedCanTransmit {
    type Error: core::error::Error;

    /// Transmit a CAN message.
    ///
    /// Returns `Err(CanError)` if encoding or bus transmission fails.
    #[allow(async_fn_in_trait)] // We'll try and maybe change it later
    async fn transmit<M: CanEncode>(&mut self, msg: M) -> Result<(), Self::Error>;
}

/// Trait for types that can receive CAN messages asynchronously.
pub trait TypedCanReceive {
    type Error: core::error::Error;

    /// Receive the next CAN message.
    ///
    /// Returns `Err(CanError)` if the frame is invalid or bus read fails.
    #[allow(async_fn_in_trait)] // We'll try and maybe change it later
    async fn recv<M: CanDecode>(&mut self) -> Result<M, Self::Error>;
}

/// Error type for CAN TX operations.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxError {
    #[error("Encoding CAN message failed")]
    Encode,

    #[error("Bug in this implementation, should be unreachable")]
    Bug,
}

/// Error type for CAN RX operations.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxError {
    #[error("CAN bus error")]
    Bus(BusError),

    #[error("Decoding CAN message failed")]
    Decode,

    #[error("Bug in the system, should be unreachable")]
    Bug,
}

impl<'a> TypedCanTransmit for CanTx<'a> {
    type Error = TxError;

    async fn transmit<M: CanEncode>(&mut self, msg: M) -> Result<(), Self::Error> {
        let mut buf = [0u8; 64];
        let (id, len) = msg.encode_into(&mut buf).map_err(|e| {
            warn!("cannot encode: {}", e);
            TxError::Encode
        })?;

        // Barring embassy changes their implementation, this will never fail.
        // since we've already ensured that the payload length is valid.
        let frame = FdFrame::new(Header::new(id.into(), len, false), &buf[..(len as usize)])
            .map_err(|_| TxError::Bug)?;

        // todo: (should we?) come up with a better way to handle dropped frames
        if let Some(pushed) = self.write_fd(&frame).await {
            warn!("CAN dropped frame: {:?}", pushed);
            // goodbye frame :(
        }

        Ok(())
    }
}
impl<'a> TypedCanReceive for CanRx<'a> {
    type Error = RxError;
    async fn recv<M: CanDecode>(&mut self) -> Result<M, Self::Error> {
        let envelope = self.read_fd().await.map_err(RxError::Bus)?;
        let frame = envelope.frame;

        let id = match frame.id() {
            Id::Standard(id) => id,
            Id::Extended(_id) => {
                // should really be unreachable, since we only use standard ids,
                // but let's stay on the safe side
                return Err(RxError::Bug);
            }
        };

        let msg = M::from_parts(*id, frame.data()).map_err(|e| {
            warn!("cannot decode: {}", e);
            RxError::Decode
        })?;
        Ok(msg)
    }
}
