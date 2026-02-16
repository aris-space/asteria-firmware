#![allow(dead_code)]

use embassy_stm32::can::enums::BusError;
use embassy_stm32::can::filter::{Action, FilterType, StandardFilter};
use embassy_stm32::can::frame::{self, FdFrame, Header};
use embassy_stm32::can::{Can, CanConfigurator, CanRx, CanTx, OperatingMode, RxPin, TxPin};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::{Peri, can};
use embassy_time::TimeoutError;
use embedded_can::Id;
use embedded_utils::fmt::*;
use hermes_can::{
    CanDecodeError, CanEncodeError, CanMessage, messages::Message, next_valid_length,
};

#[allow(unused_imports)]
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
#[cfg(not(feature = "defmt"))]
use panic_reset as _;

/// Error type for CAN operations.
#[derive(Debug, thiserror::Error)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CanError {
    #[error("CAN bus error")]
    Bus(BusError),

    #[error("CAN timeout")]
    Timeout(TimeoutError),

    #[error("Encoding CAN message failed: {0}")]
    Encode(#[from] CanEncodeError),

    #[error("Decoding CAN message failed: {0}")]
    Decode(#[from] CanDecodeError),

    #[error("Invalid frame or Id sent/received")]
    Other,
}

pub trait CanTransmitter {
    /// Transmit a CAN message.
    ///
    /// Returns `Err(CanError)` if encoding or bus transmission fails.
    async fn transmit<M: CanMessage>(&mut self, msg: M) -> Result<(), CanError>;
}

pub trait CanReceiver {
    /// Receive the next CAN message and its timestamp.
    ///
    /// Returns `Err(CanError)` if the frame is invalid or bus read fails.
    async fn recv(&mut self) -> Result<(Message, frame::Timestamp), CanError>;
}

macro_rules! impl_can_transmitter {
    ($ty:ty) => {
        impl CanTransmitter for $ty {
            async fn transmit<M: CanMessage>(&mut self, msg: M) -> Result<(), CanError> {
                let mut buf = [0u8; 64];
                let (id, len) = msg.try_write_into(&mut buf)?;

                // `next_valid_length` should never return `None`, since payload length is already checked
                // to be less than 64. This also means that the cast to `u8` is safe.
                let dlc = next_valid_length(len).ok_or(CanError::Other)?;

                // zero-pad the payload to the next valid length.
                let payload = &buf[..dlc];

                // Barring embassy changes their implementation, this will never fail.
                // since we've already ensured that the payload length is valid.
                let frame = FdFrame::new(Header::new(id.into(), dlc as u8, false), payload)
                    .map_err(|_| CanError::Other)?;

                // todo: (should we?) come up with a better way to handle dropped frames
                if let Some(pushed) = self.write_fd(&frame).await {
                    warn!("CAN dropped frame: {:?}", pushed);
                    // goodbye frame :(
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
                    Id::Extended(_id) => {
                        // should really be unreachable, since we only use standard ids,
                        // but let's stay on the safe side
                        return Err(CanError::Other);
                    }
                };

                let x = Message::try_from_parts(*id, frame.data())?;
                Ok((x, envelope.ts))
            }
        }
    };
}

impl_can_receiver!(Can<'_>);
impl_can_transmitter!(Can<'_>);
impl_can_receiver!(CanRx<'_>);
impl_can_transmitter!(CanTx<'_>);

pub fn setup_can<'a, T: can::Instance>(
    peri: Peri<'a, T>,
    rx: Peri<'a, impl RxPin<T>>,
    tx: Peri<'a, impl TxPin<T>>,
    _irqs: impl Binding<T::IT0Interrupt, can::IT0InterruptHandler<T>>
    + Binding<T::IT1Interrupt, can::IT1InterruptHandler<T>>
    + 'a,
) -> Can<'a> {
    let mut can = CanConfigurator::new(peri, rx, tx, _irqs);
    can.set_bitrate(1_000_000);
    can.set_fd_data_bitrate(1_000_000, false);

    const FILTER_COUNT: usize = 28;
    let mut filters: [StandardFilter; FILTER_COUNT] = [StandardFilter {
        filter: FilterType::Disabled,
        action: Action::Disable,
    }; FILTER_COUNT];

    // This will fail to compile if the number of enabled ids exceeds the number of filters
    const __ASSERT_LEN_OK: () = {
        if Message::NUM_ENABLED_IDS >= FILTER_COUNT - 1 {
            core::panic!("Too many receiving can ids");
        }
    };
    filters[Message::NUM_ENABLED_IDS] = StandardFilter::reject_all();

    // Configure the IDs based on the enabled messages in the `hermes-can` crate.
    for (filter_idx, id) in Message::ENABLED_IDS.iter().enumerate() {
        trace!("Setting up filter for id: {:#X}", id.as_raw());
        filters[filter_idx] = StandardFilter {
            filter: FilterType::DedicatedSingle(*id),
            action: Action::StoreInFifo1,
        };
    }
    can.properties().set_standard_filters(&filters);

    let can = can.start(OperatingMode::NormalOperationMode);

    can
}
