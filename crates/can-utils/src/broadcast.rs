use crate::rxtx::TypedCanTransmit;
use embassy_executor::{SpawnError, Spawner};
use embassy_stm32::can::CanTx;
use embassy_sync::blocking_mutex::raw::{RawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;

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
