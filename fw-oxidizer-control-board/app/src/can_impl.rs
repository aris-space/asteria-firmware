#![allow(dead_code)]
use crate::actuators::dpr::{
    DPR_CONTROL_LOOP_WATCH, DPR_PRESSURIZATION_WATCH, PRESSURIZATION_ABORT_WATCH,
    PRESSURIZATION_INFO_WATCH, PRESSURIZATION_KP,
};
use crate::actuators::valves::OXD_VENT_CONTROL;
use crate::drivers::digital_pressure::{
    DIGITAL_PRESSURE_WATCH, DPR_PRESSURE_WATCH, KELLER_BUS_ERROR_WATCH, TANK_LEVEL_WATCH,
};
use crate::drivers::temperature::THERMOCOUPLE_WATCH;
use crate::sensors::{
    CAN_BOARD_STATUS_FREQ_HZ, CAN_PRESSURE_FREQ_HZ, CAN_TANK_LEVEL_FREQ_HZ,
    CAN_TANK_TEMPERATURE_FREQ_HZ, CAN_VALVE_STATES_FREQ_HZ,
};
use embassy_executor::Spawner;
use embassy_futures::yield_now;
use embassy_stm32::can::enums::BusError;
use embassy_stm32::can::filter::{Action, FilterType, StandardFilter};
use embassy_stm32::can::frame::{self, FdFrame, Header};
use embassy_stm32::can::{Can, CanConfigurator, CanRx, CanTx, OperatingMode, RxPin, TxPin};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::{Peri, can};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, Ticker, TimeoutError, Timer, with_timeout};
use embedded_can::Id;
use embedded_utils::fmt::*;
use hermes_can::messages::board_status::SensorStatus::Online;
use hermes_can::messages::board_status::{
    OxidizerControlBoardStatus, OxidizerControlBoardValveStates,
};
use hermes_can::messages::debug_info::OxidizerControlBoardBuildInfo;
use hermes_can::messages::sensor_data::{
    FuelTankTemperature, OxidizerTankLevel, OxidizerTankPressure,
};
use hermes_can::{
    CanDecodeError, CanEncodeError, CanMessage, messages::Message, next_valid_length,
};

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

    let can = can.start(OperatingMode::NormalOperationMode);

    can
}

const THIS_BOARD_ID: hermes_can::messages::BoardId =
    hermes_can::messages::BoardId::OxidizerControlBoard;

#[embassy_executor::task]
pub async fn can_rx_task(mut can_rx: CanRx<'static>) -> ! {
    let dpr_ctrl_sender = DPR_CONTROL_LOOP_WATCH.sender();
    let oxd_vnt_sender = OXD_VENT_CONTROL.sender();

    let pressurization_sender = DPR_PRESSURIZATION_WATCH.sender();
    let pressurization_abort_sender = PRESSURIZATION_ABORT_WATCH.sender();

    loop {
        match can_rx.recv().await {
            Ok((msg, ts)) => {
                trace!("[CAN Task] Received message {:?} on bus at {:?}", msg, ts);

                match msg {
                    Message::ResetAll(_) => {
                        warn!("[CAN Task] Received ResetAll message, resetting FuelControlBoard");
                        reset_now();
                    }
                    Message::ResetSpecific(x) => {
                        if x.board_id == THIS_BOARD_ID {
                            warn!(
                                "[CAN Task] Received ResetSpecific message, resetting FuelControlBoard"
                            );
                            reset_now();
                        }
                    }
                    Message::OxidizerDprConfigFC(dpr_cmd) => {
                        // Handle FuelDprConfig message
                        trace!(
                            "[CAN Task] Received OxidizerDprConfig message from FC: {}",
                            dpr_cmd
                        );
                        dpr_ctrl_sender.send(dpr_cmd.oss_dpr);
                    }
                    Message::OxidizerDprConfigECU(dpr_cmd) => {
                        // Handle FuelDprConfig message
                        trace!(
                            "[CAN Task] Received OxidizerDprConfig message from ECU: {}",
                            dpr_cmd
                        );
                        dpr_ctrl_sender.send(dpr_cmd.oss_dpr);
                    }
                    Message::OxidizerDprConfigRFS(dpr_cmd) => {
                        // Handle FuelDprConfig message
                        trace!(
                            "[CAN Task] Received OxidizerDprConfig message from RFS: {}",
                            dpr_cmd
                        );
                        dpr_ctrl_sender.send(dpr_cmd.oss_dpr);
                    }
                    Message::OxidizerVentControlFC(oxd_cmd) => {
                        trace!(
                            "[CAN Task] Received OxidizerVentControl message from FC: {}",
                            oxd_cmd
                        );
                        // Handle FuelVentControl message
                        oxd_vnt_sender.send(oxd_cmd.oss_vnt_vlv);
                    }
                    Message::OxidizerVentControlECU(oxd_cmd) => {
                        trace!(
                            "[CAN Task] Received OxidizerVentControl message from FC: {}",
                            oxd_cmd
                        );
                        // Handle FuelVentControl message
                        oxd_vnt_sender.send(oxd_cmd.oss_vnt_vlv);
                    }
                    Message::OxidizerVentControlRFS(oxd_cmd) => {
                        trace!(
                            "[CAN Task] Received OxidizerVentControl message from FC: {}",
                            oxd_cmd
                        );
                        // Handle FuelVentControl message
                        oxd_vnt_sender.send(oxd_cmd.oss_vnt_vlv);
                    }
                    Message::OxidizerPressurization(prz_cmd) => {
                        trace!(
                            "[CAN Task] Received OxidizerPressurization message: {}",
                            prz_cmd
                        );
                        // Handle FuelPressurization message
                        pressurization_sender.send(prz_cmd);
                    }
                    Message::OxidizerPressurizationAbort(abort) => {
                        trace!(
                            "[CAN Task] Received OxidizerPressurizationAbort message: {}",
                            abort
                        );
                        // Handle FuelPressurizationAbort message
                        pressurization_abort_sender.send(abort);
                    }
                    Message::OxidizerPressurizationGain(gain) => {
                        trace!(
                            "[CAN Task] Received OxidizerPressurizationGain message: {}",
                            gain
                        );
                        // Handle FuelPressurizationGain message
                        *PRESSURIZATION_KP.lock().await = gain.kp;
                    }
                    _ => {
                        warn!("[CAN Task] Received unknown CAN message: {}", msg);
                    }
                };
            }
            Err(err) => {
                error!("CAN RX error: {:?}", err);
            }
        }
        yield_now().await;
    }
}

pub async fn spawn_can_tx_task(can_tx: CanTx<'static>, spawner: Spawner) {
    // Our transmission policy is as follows:
    // Send new data when available, but only if a minimum period has elapsed since the last
    // transmission. Otherwise, discard and wait for the next data.

    // we need to wrap CanTx in a something to allow multiple tasks to access it
    static CAN_TX: OnceLock<Mutex<NoopRawMutex, CanTx<'static>>> = OnceLock::new();

    CAN_TX
        .init(Mutex::new(can_tx))
        .ok()
        .expect("Failed to set CAN TX mutex");

    let can_tx = CAN_TX.get().await;

    // Our transmission policy is as follows:
    // Send new data when available, but only if a minimum period has elapsed since the last
    // transmission. Otherwise, discard and wait for the next data.

    // common transmit timeout
    const TX_TIMEOUT_MS: u64 = 100;
    const WATCH_TIMEOUT_MS: u64 = 5000;

    // DigitalPressure (≈20 Hz)
    #[embassy_executor::task]
    async fn pressure_task(can_tx: &'static Mutex<NoopRawMutex, CanTx<'static>>) {
        let mut pressure_watch = DIGITAL_PRESSURE_WATCH
            .receiver()
            .expect("[CAN Task] failed to get pressure watch");
        let mut dpr_pressure_watch = DPR_PRESSURE_WATCH
            .receiver()
            .expect("[CAN Task] failed to get dpr pressure watch");
        let mut ticker = Ticker::every(Duration::from_millis(
            (1000.0 / CAN_PRESSURE_FREQ_HZ) as u64,
        ));
        loop {
            let p = pressure_watch.get().await;

            let oss_tnk_p_filtered = dpr_pressure_watch.get().await;

            let oxidizer_tank_pressure = OxidizerTankPressure {
                oss_tnk_p1: p.oss_tnk_p1,
                oss_tnk_p2: p.oss_tnk_p2,
                oss_tnk_p_filtered,
            };
            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(oxidizer_tank_pressure.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        trace!(
                            "[CAN Task] Sent pressure data: {:?}",
                            oxidizer_tank_pressure
                        );
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

    // TankLevel (≈10 Hz)
    #[embassy_executor::task]
    async fn tank_level_task(can_tx: &'static Mutex<NoopRawMutex, CanTx<'static>>) {
        let mut tank_level_watch = TANK_LEVEL_WATCH
            .receiver()
            .expect("[CAN Task] failed to get tank level watch");
        let mut ticker = Ticker::every(Duration::from_millis(
            (1000.0 / CAN_TANK_LEVEL_FREQ_HZ) as u64,
        ));
        loop {
            let p = tank_level_watch.get().await;

            let data = OxidizerTankLevel {
                fill_fraction: p.oss_tnk_lvl,
            };
            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(data.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        trace!("[CAN Task] Sent tank level data: {:?}", data);
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

    // Valve States Task (≈5 Hz)
    #[embassy_executor::task]
    async fn valve_states_task(can_tx: &'static Mutex<NoopRawMutex, CanTx<'static>>) {
        let mut dpr_ctrl_watcher = DPR_CONTROL_LOOP_WATCH.receiver().unwrap();
        let mut oxd_vnt_watcher = OXD_VENT_CONTROL.receiver().unwrap();

        let mut data = OxidizerControlBoardValveStates {
            oss_dpr: Default::default(),
            oss_vnt_vlv: Default::default(),
        };

        let mut ticker = Ticker::every(Duration::from_millis(
            1000 / CAN_VALVE_STATES_FREQ_HZ as u64,
        ));
        let mut changed = false;
        loop {
            if let Some(state) = dpr_ctrl_watcher.try_changed() {
                data.oss_dpr = state;
                changed = true;
            }
            if let Some(state) = oxd_vnt_watcher.try_changed() {
                data.oss_vnt_vlv = state;
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
                        //trace!("[CAN Task] sent Valve States: {:?}", data);
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
            Timer::after_millis(10).await;
        }
    }

    // Tank Temperature Task ( ≈10 Hz)
    #[embassy_executor::task]
    async fn tank_temperature_task(can_tx: &'static Mutex<NoopRawMutex, CanTx<'static>>) {
        let mut tank_temperature_watch = THERMOCOUPLE_WATCH
            .receiver()
            .expect("[CAN Task] failed to get tanker temperature watch");
        let mut ticker = Ticker::every(Duration::from_millis(
            (1000.0 / CAN_TANK_TEMPERATURE_FREQ_HZ) as u64,
        ));
        loop {
            let p = tank_temperature_watch.get().await;

            let data = FuelTankTemperature {
                fss_tnk_t: p.fss_tnk_t,
            };
            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(data.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        trace!("[CAN Task] Sent fuel tank temperature data: {:?}", data);
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

    // Board Status Task (≈1 Hz)
    #[embassy_executor::task]
    async fn board_status_task(can_tx: &'static Mutex<NoopRawMutex, CanTx<'static>>) {
        let mut keller_watcher = KELLER_BUS_ERROR_WATCH.receiver().unwrap();

        let mut data = OxidizerControlBoardStatus {
            common: Default::default(),
            thermocouple_status: Online,
            pressure_bus: Default::default(),
        };

        let start = Instant::now();
        let mut ticker = Ticker::every(Duration::from_millis(
            1000 / CAN_BOARD_STATUS_FREQ_HZ as u64,
        ));
        loop {
            if let Some(status) = keller_watcher.try_changed() {
                data.pressure_bus = status;
            }

            data.common.micros_since_restart = (Instant::now() - start).as_micros();

            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(data.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        //trace!("[CAN Task] sent Board Status: {:?}", data);
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
    async fn build_information_task(can_tx: &'static Mutex<NoopRawMutex, CanTx<'static>>) {
        let build_info = crate::build_info::BUILD_INFO.get();
        let build_info_msg = OxidizerControlBoardBuildInfo {
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
                    Ok(Ok(())) => {
                        //trace!("Sent OxidizerControlBoardBuildInfo: {:?}", build_info_msg)
                    }
                    Ok(Err(err)) => {
                        error!("Error sending OxidizerControlBoardBuildInfo: {:?}", err)
                    }
                    Err(_) => error!(
                        "Timeout sending OxidizerControlBoardBuildInfo after {} ms",
                        TX_TIMEOUT_MS
                    ),
                }
            }

            ticker.next().await;
        }
    }

    #[embassy_executor::task]
    async fn pressurization_info_task(can_tx: &'static Mutex<NoopRawMutex, CanTx<'static>>) {
        let mut pressurization_info_watcher = PRESSURIZATION_INFO_WATCH.receiver().unwrap();

        loop {
            let info = pressurization_info_watcher.changed().await;

            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(info.clone()),
                )
                .await
                {
                    Ok(Ok(())) => trace!("Sent PressurizationInfo, {:?}", info),
                    Ok(Err(err)) => error!("Error sending PressurizationInfo: {:?}", err),
                    Err(_) => error!(
                        "Timeout sending PressurizationInfo after {} ms",
                        TX_TIMEOUT_MS
                    ),
                }
            }
            Timer::after_millis(500).await;
        }
    }

    spawner.spawn(pressure_task(can_tx)).unwrap();
    spawner.spawn(tank_level_task(can_tx)).unwrap();
    spawner.spawn(tank_temperature_task(can_tx)).unwrap();
    spawner.spawn(valve_states_task(can_tx)).unwrap();
    spawner.spawn(board_status_task(can_tx)).unwrap();
    spawner.spawn(build_information_task(can_tx)).unwrap();
    spawner.spawn(pressurization_info_task(can_tx)).unwrap();
}

/// Resets the system instantly and restarts the firmware.
fn reset_now() {
    cortex_m::peripheral::SCB::sys_reset();
}
