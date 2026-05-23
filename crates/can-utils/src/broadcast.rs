use crate::rxtx::TypedCanTransmit;
use can_hal::CanEncode;
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::can::CanTx;
#[cfg(not(target_os = "none"))] // cheat, since `ThreadModeRawMutex` only exists for cortex-m.
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as ThreadModeRawMutex;
use embassy_sync::blocking_mutex::raw::RawMutex;
#[cfg(target_os = "none")]
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Duration;
use embassy_time::Instant;

pub use broadcast_derive::Broadcast;

/// Spawns a CAN broadcast task for all tracked values in the struct.
///
/// This is best derived by [`Broadcast`][macro@Broadcast].
///
/// ## Usage
///
/// ```rust,ignore
/// #[derive(Broadcast)]
/// #[broadcast(loop_type = "PeriodicLoop")]
/// struct ValveState {
///     #[broadcast(map = "CanMessage::FuelValve(#value)", min_freq_hz = 1., max_freq_hz = 10.)]
///     fuel_valve: Watch<bool>,
///     #[broadcast(filter_map = "#value.map(CanMessage::PartialValue)", min_freq_hz = 1., max_freq_hz = 10.)]
///     partial_value: Watch<Option<i64>>,
/// }
///
/// let state = ValveState { ... };
/// state.start_broadcasting(spawner, tx)?;
/// ```
pub trait Broadcast {
    /// When derived, this function spawns generated embassy task per annotated field.
    /// Each task reads updates from the field's [`Watch`][embassy_sync::watch::Watch]
    /// and write them to `transmit` in a time aware fashion,
    /// depending on the [`BroadcastLoop`] implementation given to a macro attribute.
    fn start_broadcasting(
        &'static self,
        spawner: Spawner,
        transmit: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>,
    ) -> Result<(), SpawnError>;
}

/// Defines the timing and polling strategy for a [`Broadcast`]-driven task.
///
/// Implement this on a unit struct to control how field values are read and
/// forwarded to the telemetry channel. The macro-generated [`Broadcast`] impl
/// calls this for every annotated field.
///
/// `min_freq_hz` and `max_freq_hz` are hints for implementations that want to
/// throttle or coalesce updates; a simple change-driven loop may ignore them.
pub trait BroadcastLoop<M>: 'static {
    fn broadcast_loop<T: Send + Sync + 'static + Clone, MTX: RawMutex + Sync, const N: usize>(
        field: embassy_sync::watch::Receiver<'static, MTX, T, N>,
        map: impl FnMut(T) -> Option<M> + Send + 'static,
        transmit: &'static Mutex<MTX, impl TypedCanTransmit + Send>,
        min_freq_hz: f32,
        max_freq_hz: f32,
    ) -> impl core::future::Future<Output = NeverReturns> + Send;
}

/// A type which can't be constructed and as such a function which returns this cannot return.
pub enum NeverReturns {}

/// A [`BroadcastLoop`] implementation that sends new values as they arrive,
/// while respecting the minimum and maximum frequencies.
///
/// The way this works is by waiting for new data or a timeout based on the minimum frequency.
/// If the duration since the last sent message is long enough to respect the maximum frequency,
/// it is sent, otherwise ignored.
pub struct ResponsiveLoop;

impl<M> BroadcastLoop<M> for ResponsiveLoop
where
    M: CanEncode + embedded_utils::fmt::Format + Send,
    <M as CanEncode>::Error: embedded_utils::fmt::Format,
{
    async fn broadcast_loop<
        T: Send + Sync + 'static + Clone,
        MTX: embassy_sync::blocking_mutex::raw::RawMutex + Sync,
        const N: usize,
    >(
        mut watch: embassy_sync::watch::Receiver<'static, MTX, T, N>,
        mut filter_map: impl FnMut(T) -> Option<M> + Send + 'static,
        transmit: &'static Mutex<MTX, impl TypedCanTransmit>,
        min_freq_hz: f32,
        max_freq_hz: f32,
    ) -> NeverReturns {
        const TX_TIMEOUT: Duration = Duration::from_millis(100);

        let mut last_sent = Instant::now();
        let throttle_period = Duration::from_millis((1000. / max_freq_hz) as u64);
        let resend_period = Duration::from_millis((1000. / min_freq_hz) as u64);

        loop {
            let data = match embassy_time::with_timeout(resend_period, watch.changed()).await {
                Ok(t) => t,
                Err(_) => watch.get().await,
            };
            let Some(msg) = filter_map(data) else {
                embedded_utils::trace!("Skipping new data of type {}", core::any::type_name::<T>());
                continue;
            };

            let now = Instant::now();
            if now.checked_duration_since(last_sent).unwrap_or_default() >= throttle_period {
                let mut tx = transmit.lock().await;
                embedded_utils::trace!("Sending data: {:?}", msg);
                match embassy_time::with_timeout(TX_TIMEOUT, tx.transmit(msg)).await {
                    Ok(Ok(())) => {
                        last_sent = now;
                    }
                    Ok(Err(err)) => embedded_utils::error!("CAN TX error: {:?}", err),
                    Err(_) => embedded_utils::error!(
                        "CAN TX timed out after {} ms",
                        TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                embedded_utils::trace!("Discarding data: {:?}", msg);
            }
        }
    }
}
