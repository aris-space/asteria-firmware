//! This module provides a way to setup a Can abstraction from hardware pins.

use embassy_stm32::can::filter::{Action, FilterType, StandardFilter};
use embassy_stm32::can::{Can, CanConfigurator, CanTx, OperatingMode, RxPin, TxPin};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::{Peri, can};
#[cfg(not(target_os = "none"))] // cheat, since `ThreadModeRawMutex` only exists for cortex-m.
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as ThreadModeRawMutex;
#[cfg(target_os = "none")]
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;

/// Sets up a can instance on the given pins and configures it to only receive messages with the given ids.
pub fn setup_can<'a, T: can::Instance>(
    peri: Peri<'a, T>,
    rx: Peri<'a, impl RxPin<T>>,
    tx: Peri<'a, impl TxPin<T>>,
    irqs: impl Binding<T::IT0Interrupt, can::IT0InterruptHandler<T>>
    + Binding<T::IT1Interrupt, can::IT1InterruptHandler<T>>
    + 'a,
    enabled_ids: &'static [embedded_can::StandardId],
) -> Can<'a> {
    let mut can = CanConfigurator::new(peri, rx, tx, irqs);
    can.set_bitrate(1_000_000);
    can.set_fd_data_bitrate(1_000_000, false);

    const FILTER_COUNT: usize = 28;
    let mut filters: [StandardFilter; FILTER_COUNT] = [StandardFilter {
        filter: FilterType::Disabled,
        action: Action::Disable,
    }; FILTER_COUNT];

    filters[enabled_ids.len()] = StandardFilter::reject_all();

    // Configure the IDs based on the enabled messages in the `hermes-can` crate.
    for (filter_idx, id) in enabled_ids.iter().enumerate() {
        // trace!("Setting up filter for id: {:#X}", id.as_raw());
        filters[filter_idx] = StandardFilter {
            #[allow(clippy::clone_on_copy)]
            filter: FilterType::DedicatedSingle(id.clone()),
            action: Action::StoreInFifo1,
        };
    }
    can.properties().set_standard_filters(&filters);

    /* todo: unsure if this works, did not work in last year's project
    let mut config = can.config();
    config.global_filter = GlobalFilter::accept_all();
    can.set_config(config);
     */
    can.start(OperatingMode::NormalOperationMode)
}

/// Moves the can tx instance behind a singleton mutex.
///
/// Only call this once! And only use this on single-core MCUs.
pub async fn make_multiplexable(
    can_tx: CanTx<'static>,
) -> &'static Mutex<ThreadModeRawMutex, CanTx<'static>> {
    let can_tx = Mutex::<ThreadModeRawMutex, _>::new(can_tx);
    static CAN_TX: OnceLock<Mutex<ThreadModeRawMutex, CanTx<'static>>> = OnceLock::new();
    CAN_TX
        .init(can_tx)
        .ok()
        .expect("Failed to set CAN TX mutex");
    CAN_TX.get().await
}
