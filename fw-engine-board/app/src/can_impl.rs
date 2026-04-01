use crate::controls::runner::{ABORT_INITIATION, FIRING_INFO, FIRING_INITIATION, FiringInfo};
use crate::controls::{THRUST_CURVE_HASH, initiate_thrust_curve};
use crate::drivers::IGNITER_P_WATCH;
use crate::drivers::digital_pressure::DIGITAL_PRESSURE_WATCH;
use crate::drivers::temperature::{THERMOCOUPLE_ERROR_WATCH, THERMOCOUPLE_WATCH};
use crate::indicate_critical_error;
use crate::k23_temperature_control::HEATING_CONTROL_ACTIVE;
use crate::sensors::{CAN_PRESSURE_FREQ_HZ, CAN_THERMOCOUPLE_FREQ_HZ};
use crate::valves::{
    EXTERNAL_VALVE_CONTROL, ExternalValve, FSS_MAIN_CONTROL, MAIN_ARMING, OSS_MAIN_CONTROL,
};
use core::sync::atomic::Ordering;
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_stm32::can::enums::BusError;
use embassy_stm32::can::filter::{Action, FilterType, StandardFilter};
use embassy_stm32::can::frame::{self, FdFrame, Header};
use embassy_stm32::can::{Can, CanConfigurator, CanRx, CanTx, OperatingMode, RxPin, TxPin};
use embassy_stm32::gpio::Output;
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::{Peri, can};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Ticker, TimeoutError, Timer, with_timeout};
use embedded_can::Id;
use embedded_utils::fmt::*;
use hermes_can::messages::board_status::{
    ArmingState, EngineControlBoardStatus, EngineControlBoardValveStates,
};
use hermes_can::messages::event_messages::{
    CombustionDetected, FiringAborted, FiringCompleted, FiringInitiated, FuelDprConfigECU,
    FuelVentControlECU, GoxIgniterValveControl, H2IgniterValveControl, IgniterPurgeControl,
    IgniterSparkPlugControl, IgnitionDetected, OxidizerDprConfigECU, OxidizerVentControlECU,
    PressurizationVentControlECU,
};
use hermes_can::messages::sensor_data::{EngineBayTemperature, EnginePressure};
use hermes_can::{
    CanDecodeError, CanEncodeError, CanMessage, messages::Message, next_valid_length,
};

// common transmit timeout in milliseconds
const TX_TIMEOUT_MS: u64 = 100;

/// Error type for CAN operations.
#[allow(unused)]
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

/// Trait for types that can transmit CAN messages asynchronously.
pub trait CanTransmitter {
    /// Transmit a CAN message.
    ///
    /// Returns `Err(CanError)` if encoding or bus transmission fails.
    async fn transmit<M: CanMessage>(&mut self, msg: M) -> Result<(), CanError>;
}

/// Trait for types that can receive CAN messages asynchronously.
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

    can.start(OperatingMode::NormalOperationMode)
}

const THIS_BOARD_ID: hermes_can::messages::BoardId =
    hermes_can::messages::BoardId::EngineControlBoard;

// Receiving and handling CAN messages
#[embassy_executor::task]
pub async fn can_rx_task(mut can_rx: CanRx<'static>, mut yellow: Output<'static>) -> ! {
    let fss_mnl_sender = FSS_MAIN_CONTROL.sender();
    let oss_mnl_sender = OSS_MAIN_CONTROL.sender();
    let firing_initiation_sender = FIRING_INITIATION.sender();
    let abort_initiation_sender = ABORT_INITIATION.sender();
    let igniter_pressure_sender = IGNITER_P_WATCH.sender();

    loop {
        match can_rx.recv().await {
            Ok((msg, _ts)) => {
                match msg {
                    Message::ResetAll(_) => {
                        warn!(
                            "[CAN Task] Received ResetAll message, resetting Engine Control Board"
                        );
                        reset_now();
                    }
                    Message::ResetSpecific(x) => {
                        if x.board_id == THIS_BOARD_ID {
                            warn!(
                                "[CAN Task] Received ResetSpecific message, resetting Engine Control Board"
                            );
                            reset_now();
                        }
                    }
                    // Firing Messages
                    // Firing Initiation Message
                    Message::FiringInitiation(init) => {
                        // Handle FiringInitiation message
                        trace!("[CAN Task] Received FiringInitiation message");
                        firing_initiation_sender.send(init);

                        HEATING_CONTROL_ACTIVE.store(false, Ordering::Relaxed);
                    }
                    // Firing Abort Initiation Message
                    Message::FiringAbortInitiation(init) => {
                        // Handle FiringAbortInitiation message
                        trace!("[CAN Task] Received FiringAbortInitiation message");
                        abort_initiation_sender.send(init);
                    }
                    // Thrust Curve Configuration Message
                    Message::ThrustCurveConfig(cfg) => {
                        // Handle ThrustCurveConfig message
                        info!("[CAN Task] Received ThrustCurveConfig message: {:?}", cfg);

                        match initiate_thrust_curve(cfg).await {
                            Ok(hash) => {
                                info!("[CAN Task] Thrust curve set with hash: {}", hash);
                                *THRUST_CURVE_HASH.lock().await = hash;
                            }
                            Err(e) => {
                                error!("[CAN Task] Failed to initiate thrust curve: {}", e);
                                indicate_critical_error().await;
                            }
                        }
                    }
                    // Valve Control Messages
                    // Fuel Main Control
                    Message::FuelMainControlFC(cmd) => {
                        // Handle FuelMainControlFC message
                        trace!("[CAN Task] Received FuelMainControlFC message: {:?}", cmd);
                        fss_mnl_sender.send(cmd.fss_mnl_vlv);
                    }
                    Message::FuelMainControlRFS(cmd) => {
                        // Handle FuelMainControlRFS message
                        trace!("[CAN Task] Received FuelMainControlRFS message: {:?}", cmd);
                        fss_mnl_sender.send(cmd.fss_mnl_vlv);
                    }

                    // Oxidizer Main Control
                    Message::OxidizerMainControlFC(cmd) => {
                        // Handle OxidizerMainControlFC message
                        trace!(
                            "[CAN Task] Received OxidizerMainControlFC message: {:?}",
                            cmd
                        );
                        oss_mnl_sender.send(cmd.oss_mnl_vlv);
                    }
                    Message::OxidizerMainControlRFS(cmd) => {
                        // Handle OxidizerMainControlRFS message
                        trace!(
                            "[CAN Task] Received OxidizerMainControlRFS message: {:?}",
                            cmd
                        );
                        oss_mnl_sender.send(cmd.oss_mnl_vlv);
                    }
                    Message::IgniterPressure(data) => {
                        // trace!("[CAN Task] Received IgniterPressure message: {:?}", data);
                        igniter_pressure_sender.send(data.igniter_pressure);
                    }
                    _ => {
                        warn!("[CAN Task] Received unknown CAN message: {:?}", msg);
                    }
                };
                yellow.toggle();
            }
            Err(err) => {
                error!("CAN RX error: {:?}", err);
            }
        }

        // Make sure task yields back to the executor to ensure other tasks can still run
        yield_now().await;
    }
}

/// # Transmitting CAN messages
pub async fn spawn_can_tx_task(can_tx: CanTx<'static>, spawner: Spawner) {
    // Our transmission policy is as follows:
    // Send new data when available, but only if a minimum period has elapsed since the last
    // transmission. Otherwise, discard and wait for the next data.

    // we need to wrap CanTx in a something to allow multiple tasks to access it
    static CAN_TX: OnceLock<Mutex<ThreadModeRawMutex, CanTx<'static>>> = OnceLock::new();

    CAN_TX
        .init(Mutex::new(can_tx))
        .ok()
        .expect("Failed to set CAN TX mutex");

    let can_tx = CAN_TX.get().await;

    const CAN_VALVE_STATES_FREQ: u32 = 5; // Hz
    const CAN_BOARD_STATUS_FREQ_HZ: u32 = 1; // Hz

    // AnalogPressure (≈20 Hz)
    #[embassy_executor::task]
    async fn pressure_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        // Using digital pressure sensors
        // let mut pressure_watch = ANALOG_PRESSURE_WATCH.receiver().unwrap();
        let mut pressure_watch = DIGITAL_PRESSURE_WATCH.receiver().unwrap();

        let mut ticker = Ticker::every(Duration::from_millis(
            (1000.0 / CAN_PRESSURE_FREQ_HZ) as u64,
        ));

        loop {
            let p = pressure_watch.get().await;

            let can_data = EnginePressure {
                eng_cc_p: p.eng_cc_p,
                fss_inj_p: p.fue_inj_p,
                oss_inj_p: p.oxd_inj_p,
            };
            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(can_data.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        trace!("[CAN Task] sent pressure data: {:?}", can_data);
                    }
                    Ok(Err(err)) => {
                        error!("[CAN Task] CAN TX error: {:?}", err);
                    }
                    Err(_) => {
                        error!("[CAN Task] CAN TX timed out after {} ms", TX_TIMEOUT_MS);
                    }
                }
            }
            ticker.next().await;
        }
    }

    // Thermocouple Task (≈10 Hz)
    #[embassy_executor::task]
    async fn thermocouple_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut thermocouple_watch = THERMOCOUPLE_WATCH.receiver().unwrap();

        let mut ticker = Ticker::every(Duration::from_millis(
            (1000.0 / CAN_THERMOCOUPLE_FREQ_HZ) as u64,
        ));

        loop {
            let t = thermocouple_watch.get().await;

            let can_data = EngineBayTemperature {
                oss_rnl_t: t.oss_rnl_t,
                oss_tnk_t: t.oss_tnk_t,
            };

            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(can_data.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        trace!("[CAN Task] sent thermocouple data: {:?}", can_data);
                    }
                    Ok(Err(err)) => {
                        error!("CAN TX error: {:?}", err);
                    }
                    Err(_) => {
                        error!("CAN TX timed out after {} ms", TX_TIMEOUT_MS);
                    }
                }
            }
            ticker.next().await;
        }
    }

    // Valve States Task (≈5 Hz)
    #[embassy_executor::task]
    async fn valve_states_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut fss_mnl_watcher = FSS_MAIN_CONTROL.receiver().unwrap();
        let mut oss_mnl_watcher = OSS_MAIN_CONTROL.receiver().unwrap();

        let mut data = EngineControlBoardValveStates {
            fss_mnl_vlv: Default::default(),
            oss_mnl_vlv: Default::default(),
        };

        let mut ticker = Ticker::every(Duration::from_millis(1000 / CAN_VALVE_STATES_FREQ as u64));
        let mut changed = false;
        loop {
            if let Some(state) = fss_mnl_watcher.try_changed() {
                data.fss_mnl_vlv = state;
                changed = true;
            }
            if let Some(state) = oss_mnl_watcher.try_changed() {
                data.oss_mnl_vlv = state;
                changed = true;
            }

            if !changed {
                ticker.next().await;
            }
            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(data.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        trace!("[CAN Task] sent Valve States: {:?}", data);
                        changed = false;
                    }
                    Ok(Err(err)) => {
                        error!("CAN TX error: {:?}", err);
                    }
                    Err(_) => {
                        error!("CAN TX timed out after {} ms", TX_TIMEOUT_MS);
                    }
                }
            }
        }
    }

    // External Valve Command Task (non-periodic)
    #[embassy_executor::task]
    async fn external_valve_command_task(
        can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>,
    ) {
        let mut general_valve_watcher = EXTERNAL_VALVE_CONTROL.subscriber().unwrap();

        loop {
            let cmd = general_valve_watcher.next_message().await;
            match cmd {
                WaitResult::Lagged(_) => {
                    warn!("Lagged while waiting for External Valve Command, command was missed");
                }
                WaitResult::Message(cmd) => {
                    let mut tx = can_tx.lock().await;
                    match with_timeout(Duration::from_millis(TX_TIMEOUT_MS), async {
                        match cmd {
                            ExternalValve::NitrogenVent(state) => {
                                if let Err(e) = tx
                                    .transmit(PressurizationVentControlECU { prz_vnt_vlv: state })
                                    .await
                                {
                                    error!("CAN TX error: {:?}", e);
                                }
                            }
                            ExternalValve::FuelDpr(state) => {
                                if let Err(e) =
                                    tx.transmit(FuelDprConfigECU { fss_dpr: state }).await
                                {
                                    error!("CAN TX error: {:?}", e);
                                }
                            }
                            ExternalValve::FuelVent(state) => {
                                if let Err(e) =
                                    tx.transmit(FuelVentControlECU { fss_vnt_vlv: state }).await
                                {
                                    error!("CAN TX error: {:?}", e);
                                }
                            }
                            ExternalValve::OxidizerDpr(state) => {
                                if let Err(e) =
                                    tx.transmit(OxidizerDprConfigECU { oss_dpr: state }).await
                                {
                                    error!("CAN TX error: {:?}", e);
                                }
                            }
                            ExternalValve::OxidizerVent(state) => {
                                if let Err(e) = tx
                                    .transmit(OxidizerVentControlECU { oss_vnt_vlv: state })
                                    .await
                                {
                                    error!("CAN TX error: {:?}", e);
                                }
                            }
                            ExternalValve::IgniterFuel(state) => {
                                if let Err(e) = tx
                                    .transmit(H2IgniterValveControl { h2_ign_vlv: state })
                                    .await
                                {
                                    error!("CAN TX error: {:?}", e);
                                }
                            }
                            ExternalValve::IgniterOxidizer(state) => {
                                if let Err(e) = tx
                                    .transmit(GoxIgniterValveControl { gox_ign_vlv: state })
                                    .await
                                {
                                    error!("CAN TX error: {:?}", e);
                                }
                            }
                            ExternalValve::IgniterPurge(state) => {
                                if let Err(e) = tx
                                    .transmit(IgniterPurgeControl { gox_prg_vlv: state })
                                    .await
                                {
                                    error!("CAN TX error: {:?}", e);
                                }
                            }
                            ExternalValve::IgniterSpark(state) => {
                                if let Err(e) = tx
                                    .transmit(IgniterSparkPlugControl { ign_spk_plg: state })
                                    .await
                                {
                                    error!("CAN TX error: {:?}", e);
                                }
                            }
                        }
                    })
                    .await
                    {
                        Ok(_) => {
                            trace!("[CAN Task] sent External Valve Command: {:?}", cmd);
                        }
                        Err(_) => {
                            error!("CAN TX timed out after {} ms", TX_TIMEOUT_MS);
                        }
                    }
                }
            }
            // Make sure that if many commands are incoming at the same time the loop doesn't block other tasks
            yield_now().await;
        }
    }

    // Board Status Task (≈1 Hz)
    #[embassy_executor::task]
    async fn board_status_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut thermocouple_watcher = THERMOCOUPLE_ERROR_WATCH.receiver().unwrap();
        let start = Instant::now();
        let mut data = EngineControlBoardStatus {
            common: Default::default(),
            thermocouple_status: Default::default(),
            thrust_curve_hash: 0,
            armed: ArmingState::Armed,
        };

        let mut ticker = Ticker::every(Duration::from_millis(
            1000 / CAN_BOARD_STATUS_FREQ_HZ as u64,
        ));
        loop {
            {
                data.thrust_curve_hash = *THRUST_CURVE_HASH.lock().await;
                data.armed = MAIN_ARMING.lock().await.clone();
                if let Some(status) = thermocouple_watcher.try_changed() {
                    data.thermocouple_status = status;
                }
                data.common.micros_since_restart = start.elapsed().as_micros();
            }
            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(data.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        trace!("[CAN Task] sent Board Status Command: {:?}", data);
                    }
                    Ok(Err(err)) => {
                        error!("CAN TX error: {:?}", err);
                    }
                    Err(_) => {
                        error!("CAN TX timed out after {} ms", TX_TIMEOUT_MS);
                    }
                }
            }
            ticker.next().await;
        }
    }

    // Build Information Task (≈0.2 Hz)
    #[embassy_executor::task]
    async fn build_information_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let build_info = crate::build_info::BUILD_INFO.get();
        let build_info_msg = hermes_can::messages::debug_info::EngineControlBoardBuildInfo {
            data: build_info.clone(),
        };

        let mut ticker = Ticker::every(Duration::from_secs(5));
        loop {
            // this scope is necessary to ensure the lock is released – do not remove it!
            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(build_info_msg.clone()),
                )
                .await
                {
                    Ok(Ok(())) => trace!("Sent EngineControlBoardBuildInfo: {:?}", build_info_msg),
                    Ok(Err(err)) => error!("Error sending EngineControlBoardBuildInfo: {:?}", err),
                    Err(_) => error!(
                        "Timeout sending EngineControlBoardBuildInfo after {} ms",
                        TX_TIMEOUT_MS
                    ),
                }
            }

            ticker.next().await;
        }
    }

    #[embassy_executor::task]
    async fn firing_info_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut firing_info_watcher = FIRING_INFO.subscriber().unwrap();

        loop {
            if let Some(info) = firing_info_watcher.try_next_message_pure() {
                // info!("[FIRING CAN] Firing Info: {:?}", info);

                let mut tx = can_tx.lock().await;
                match with_timeout(Duration::from_millis(TX_TIMEOUT_MS), async {
                    match info {
                        FiringInfo::FiringCompleted => tx.transmit(FiringCompleted).await,
                        FiringInfo::FiringAborted => tx.transmit(FiringAborted).await,
                        FiringInfo::FiringInitiated => tx.transmit(FiringInitiated).await,
                        FiringInfo::IgnitionDetected => tx.transmit(IgnitionDetected).await,
                        FiringInfo::CombustionDetected => tx.transmit(CombustionDetected).await,
                    }
                })
                .await
                {
                    Ok(Ok(())) => trace!("Sent FiringInfo, {:?}", info),
                    Ok(Err(err)) => error!("Error sending FiringInfo: {:?}", err),
                    Err(_) => error!("Timeout sending FiringInfo after {} ms", TX_TIMEOUT_MS),
                }
            }
            Timer::after_millis(100).await;
        }
    }

    // Run all tasks concurrently
    spawner
        .spawn(pressure_task(can_tx))
        .expect("Failed to spawn pressure_task");
    spawner
        .spawn(thermocouple_task(can_tx))
        .expect("Failed to spawn thermocouple_task");
    spawner
        .spawn(valve_states_task(can_tx))
        .expect("Failed to spawn valve_states_task");
    spawner
        .spawn(external_valve_command_task(can_tx))
        .expect("Failed to spawn external_valve_command_task");
    spawner
        .spawn(board_status_task(can_tx))
        .expect("Failed to spawn board_status_task");
    spawner
        .spawn(build_information_task(can_tx))
        .expect("Failed to spawn build_information_task");
    spawner
        .spawn(firing_info_task(can_tx))
        .expect("Failed to spawn firing info_task");
}

/// Resets the system instantly and restarts the firmware.
fn reset_now() {
    cortex_m::peripheral::SCB::sys_reset();
}
