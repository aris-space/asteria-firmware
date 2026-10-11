use core::future::{Future, poll_fn};

use can_utils::rxtx::{RxError, TxError};
use data_core::can::hal::{CanDecode, CanEncode};
use embassy_stm32::can::{
    BufferedFdCanReceiver, BufferedFdCanSender,
    frame::{FdFrame, Header},
};
use embedded_can::Id;

// TODO: Move these helpers into can-utils and unify.

/// Nonblocking typed CAN transmission.
pub trait BufferedCanTransmit {
    type Error: core::error::Error + defmt::Format;

    /// Queue a CAN message without waiting for capacity.
    ///
    /// Fails on encoding errors, invalid frames, or a full TX buffer.
    fn try_transmit<M: CanEncode>(&mut self, msg: M) -> Result<(), Self::Error>;
}

/// Asynchronous typed CAN reception.
pub trait BufferedCanReceive {
    type Error: core::error::Error + defmt::Format;

    /// Receive and decode the next CAN message.
    ///
    /// Fails on bus errors, extended IDs, or decoding errors.
    fn recv<M: CanDecode>(&mut self) -> impl Future<Output = Result<M, Self::Error>> + Send;
}

#[derive(Debug, defmt::Format)]
pub enum TryTxError {
    Tx(TxError),
    Full,
}

impl core::fmt::Display for TryTxError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Tx(err) => write!(f, "{err}"),
            Self::Full => f.write_str("CAN TX buffer full"),
        }
    }
}

impl core::error::Error for TryTxError {}

impl From<TxError> for TryTxError {
    fn from(err: TxError) -> Self {
        Self::Tx(err)
    }
}

impl BufferedCanTransmit for BufferedFdCanSender {
    type Error = TryTxError;

    fn try_transmit<M: CanEncode>(&mut self, msg: M) -> Result<(), Self::Error> {
        let mut buf = [0u8; 64];
        let (id, len) = msg.encode_into(&mut buf).map_err(|_| TxError::Encode)?;
        let payload = buf.get(..usize::from(len)).ok_or(TxError::Bug)?;
        let frame =
            FdFrame::new(Header::new(id.into(), len, false), payload).map_err(|_| TxError::Bug)?;
        self.try_write(frame).map_err(|_| TryTxError::Full)
    }
}

impl BufferedCanReceive for BufferedFdCanReceiver {
    type Error = RxError;

    async fn recv<M: CanDecode>(&mut self) -> Result<M, Self::Error> {
        let envelope = poll_fn(|cx| self.poll_receive(cx))
            .await
            .map_err(RxError::Bus)?;
        let frame = envelope.frame;
        let Id::Standard(id) = frame.id() else {
            return Err(RxError::Bug);
        };
        M::from_parts(*id, frame.data()).map_err(|_| RxError::Decode)
    }
}
