//! This modules defines and implements traits to received typed CAN messages.

use can_hal::{CanDecode, CanEncode};
use embassy_stm32::can::{
    BufferedFdCanReceiver, BufferedFdCanSender, CanRx, CanTx,
    enums::BusError,
    frame::{FdFrame, Header},
};
use embedded_can::Id;
use embedded_utils::fmt::{Format, warn};

/// Trait for types that can transmit CAN messages asynchronously.
pub trait TypedCanTransmit {
    type Error: core::error::Error + Format;

    /// Transmit a CAN message.
    ///
    /// Returns `Err(CanError)` if encoding or bus transmission fails.
    fn transmit<M: CanEncode + Send>(
        &mut self,
        msg: M,
    ) -> impl core::future::Future<Output = Result<(), Self::Error>> + Send
    where
        M::Error: Format;
}

/// Trait for types that can receive CAN messages asynchronously.
pub trait TypedCanReceive {
    type Error: core::error::Error + Format;

    /// Receive the next CAN message.
    ///
    /// Returns `Err(CanError)` if the frame is invalid or bus read fails.
    fn recv<M: CanDecode>(
        &mut self,
    ) -> impl core::future::Future<Output = Result<M, Self::Error>> + Send
    where
        M::Error: Format;
}

/// Error type for CAN TX operations.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxError {
    #[error("Encoding CAN message failed")]
    Encode,

    #[error("TX buffer full, the bus takes no frames")]
    BufferFull,

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

/// Encodes `msg` into one CAN FD frame.
fn encode<M: CanEncode>(msg: M) -> Result<FdFrame, TxError>
where
    M::Error: Format,
{
    let mut buf = [0u8; 64];
    let (id, len) = msg.encode_into(&mut buf).map_err(|_e| {
        // warn!("cannot encode: {}", e); // FIXME: e is not defmt::Format
        TxError::Encode
    })?;

    // Barring embassy changes their implementation, this will never fail.
    // since we've already ensured that the payload length is valid.
    FdFrame::new(Header::new(id.into(), len, false), &buf[..(len as usize)])
        .map_err(|_| TxError::Bug)
}

/// Decodes the message in `frame`.
fn decode<M: CanDecode>(frame: &FdFrame) -> Result<M, RxError>
where
    M::Error: Format,
{
    let id = match frame.id() {
        Id::Standard(id) => id,
        Id::Extended(_id) => {
            // should really be unreachable, since we only use standard ids,
            // but let's stay on the safe side
            return Err(RxError::Bug);
        }
    };

    M::from_parts(*id, frame.data()).map_err(|_e| {
        // warn!("cannot decode: {}", e); // FIXME: e is not defmt::Format
        RxError::Decode
    })
}

impl<'a> TypedCanTransmit for CanTx<'a> {
    type Error = TxError;

    async fn transmit<M: CanEncode>(&mut self, msg: M) -> Result<(), Self::Error>
    where
        M::Error: Format,
    {
        let frame = encode(msg)?;

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
    async fn recv<M: CanDecode>(&mut self) -> Result<M, Self::Error>
    where
        M::Error: Format,
    {
        let envelope = self.read_fd().await.map_err(RxError::Bus)?;
        decode(&envelope.frame)
    }
}

/// A typed handle to the TX buffer of a CAN peripheral in buffered FD mode,
/// see [`Can::buffered_fd`](embassy_stm32::can::Can::buffered_fd).
///
/// Every task can own a clone and send without waiting on any other task:
/// frames go into the buffer, and the driver's interrupt moves them into the
/// hardware. With no bus taking frames the buffer fills, and sends fail with
/// [`TxError::BufferFull`] instead of waiting.
#[derive(Clone)]
pub struct TypedCanSender(BufferedFdCanSender);

impl From<BufferedFdCanSender> for TypedCanSender {
    fn from(sender: BufferedFdCanSender) -> Self {
        Self(sender)
    }
}

impl TypedCanSender {
    /// Encodes `msg` and puts it into the TX buffer, without waiting.
    pub fn try_transmit<M: CanEncode>(&mut self, msg: M) -> Result<(), TxError>
    where
        M::Error: Format,
    {
        let frame = encode(msg)?;
        self.0.try_write(frame).map_err(|_| TxError::BufferFull)
    }
}

/// A typed handle to the RX buffer of a CAN peripheral in buffered FD mode,
/// the receiving counterpart of [`TypedCanSender`].
pub struct TypedCanReceiver(BufferedFdCanReceiver);

impl From<BufferedFdCanReceiver> for TypedCanReceiver {
    fn from(receiver: BufferedFdCanReceiver) -> Self {
        Self(receiver)
    }
}

impl TypedCanReceiver {
    /// Waits for the next frame and decodes it.
    pub async fn recv<M: CanDecode>(&mut self) -> Result<M, RxError>
    where
        M::Error: Format,
    {
        let envelope = self.0.receive().await.map_err(RxError::Bus)?;
        decode(&envelope.frame)
    }
}
