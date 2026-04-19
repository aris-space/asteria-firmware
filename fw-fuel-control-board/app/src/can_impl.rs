use crate::globals::STATE;
use crate::sensors::{CAN_BOARD_STATUS_FREQ_HZ, CAN_PRESSURE_FREQ_HZ, CAN_VALVE_STATES_FREQ_HZ};
use core::future::pending;
use core::panic;
use embassy_futures::join::join5;
use embassy_futures::yield_now;
use embassy_stm32::can::enums::BusError;
use embassy_stm32::can::filter::{Action, FilterType, StandardFilter};
use embassy_stm32::can::frame::{self, FdFrame, Header};
use embassy_stm32::can::{Can, CanConfigurator, CanRx, CanTx, OperatingMode, RxPin, TxPin};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::{Peri, can};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, TimeoutError, Timer, with_timeout};
use embedded_can::Id;
use embedded_utils::fmt::*;
use hermes_can::messages::board_status::SensorStatus::Online;
use hermes_can::messages::board_status::{FuelControlBoardStatus, FuelControlBoardValveStates};
use hermes_can::messages::sensor_data::{FuelTankPressure, PressurizationLinePressure};
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
            panic!("Too many receiving can ids");
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
    hermes_can::messages::BoardId::FuelControlBoard;

#[embassy_executor::task]
pub async fn can_rx_task(mut can_rx: CanRx<'static>) -> ! {
    let dpr_ctrl_sender = STATE.dpr_control_loop.sender();
    let prz_vnt_sender = STATE.pressurization_vent_control.sender();
    let fuel_vnt_sender = STATE.fuel_vent_control.sender();
    let pressurization_sender = STATE.dpr_pressurization.sender();
    let pressurization_abort_sender = STATE.pressurization_abort.sender();

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
                    Message::FuelDprConfigFC(dpr_cmd) => {
                        // Handle FuelDprConfig message
                        trace!(
                            "[CAN Task] Received FuelPressureControl message from FC: {}",
                            dpr_cmd
                        );
                        dpr_ctrl_sender.send(dpr_cmd.fss_dpr);
                    }
                    Message::FuelDprConfigECU(dpr_cmd) => {
                        // Handle FuelDprConfig message
                        trace!(
                            "[CAN Task] Received FuelPressureControl message from ECU: {}",
                            dpr_cmd
                        );
                        dpr_ctrl_sender.send(dpr_cmd.fss_dpr);
                    }
                    Message::FuelDprConfigRFS(dpr_cmd) => {
                        // Handle FuelDprConfig message
                        trace!(
                            "[CAN Task] Received FuelPressureControl message from RFS: {}",
                            dpr_cmd
                        );
                        dpr_ctrl_sender.send(dpr_cmd.fss_dpr);
                    }
                    Message::PressurizationVentControlFC(prz_cmd) => {
                        trace!(
                            "[CAN Task] Received PressureVentControl message from FC: {}",
                            prz_cmd
                        );
                        // Handle PressureVentControl message
                        prz_vnt_sender.send(prz_cmd.prz_vnt_vlv);
                    }
                    Message::PressurizationVentControlECU(prz_cmd) => {
                        trace!(
                            "[CAN Task] Received PressureVentControl message from ECU: {}",
                            prz_cmd
                        );
                        // Handle PressureVentControl message
                        prz_vnt_sender.send(prz_cmd.prz_vnt_vlv);
                    }
                    Message::PressurizationVentControlRFS(prz_cmd) => {
                        trace!(
                            "[CAN Task] Received PressureVentControl message from RFS: {}",
                            prz_cmd
                        );
                        // Handle PressureVentControl message
                        prz_vnt_sender.send(prz_cmd.prz_vnt_vlv);
                    }
                    Message::FuelVentControlFC(fuel_cmd) => {
                        trace!(
                            "[CAN Task] Received FuelVentControl message from FC: {}",
                            fuel_cmd
                        );
                        // Handle FuelVentControl message
                        fuel_vnt_sender.send(fuel_cmd.fss_vnt_vlv);
                    }
                    Message::FuelVentControlECU(fuel_cmd) => {
                        trace!(
                            "[CAN Task] Received FuelVentControl message from ECU: {}",
                            fuel_cmd
                        );
                        // Handle FuelVentControl message
                        fuel_vnt_sender.send(fuel_cmd.fss_vnt_vlv);
                    }
                    Message::FuelVentControlRFS(fuel_cmd) => {
                        trace!(
                            "[CAN Task] Received FuelVentControl message from RFS: {}",
                            fuel_cmd
                        );
                        // Handle FuelVentControl message
                        fuel_vnt_sender.send(fuel_cmd.fss_vnt_vlv);
                    }
                    Message::FuelPressurization(prz_cmd) => {
                        trace!(
                            "[CAN Task] Received FuelPressurization message: {}",
                            prz_cmd
                        );
                        pressurization_sender.send(prz_cmd);
                    }
                    Message::FuelPressurizationAbort(abort) => {
                        trace!(
                            "[CAN Task] Received FuelPressurizationAbort message: {}",
                            abort
                        );
                        pressurization_abort_sender.send(abort);
                    }
                    Message::FuelPressurizationGain(gain) => {
                        trace!(
                            "[CAN Task] Received FuelPressurizationGain message: {}",
                            gain
                        );

                        *STATE.pressurization_kp.lock().await = gain.kp;
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

#[embassy_executor::task]
pub async fn can_tx_task(can_tx: CanTx<'static>) -> ! {
    // Our transmission policy is as follows:
    // Send new data when available, but only if a minimum period has elapsed since the last
    // transmission. Otherwise, discard and wait for the next data.

    // we need to wrap CanTx in a something to allow multiple tasks to access it
    let can_tx: Mutex<NoopRawMutex, _> = Mutex::new(can_tx);

    // common transmit timeout
    const TX_TIMEOUT_MS: u64 = 100;
    const WATCH_TIMEOUT_MS: u64 = 5000;

    // AnalogPressure (≈20 Hz)
    let pressure_task = async {
        let mut ticker = Ticker::every(Duration::from_millis(1000 / CAN_PRESSURE_FREQ_HZ as u64));

        let mut prz_mnl_p_watch = STATE
            .pressurization_pressure
            .receiver()
            .expect("[CAN Task] failed to get PRESSURIZATION_COPV_PRESSURE watch");

        let mut fss_tnk_p_watch = STATE
            .fuel_tank_pressure
            .receiver()
            .expect("[CAN Task] failed to get FUEL_TANK_PRESSURE watch");

        loop {
            let prz_mnl_p = loop {
                if let Ok(p) = with_timeout(
                    Duration::from_millis(WATCH_TIMEOUT_MS),
                    prz_mnl_p_watch.changed(),
                )
                .await
                {
                    break p;
                }
                error!("[CAN Task] Timeout waiting for PRESSURIZATION_COPV_PRESSURE data");
            };

            let fss_tnk_p = loop {
                if let Ok(p) = with_timeout(
                    Duration::from_millis(WATCH_TIMEOUT_MS),
                    fss_tnk_p_watch.changed(),
                )
                .await
                {
                    break p;
                }
                error!("[CAN Task] Timeout waiting for FUEL_TANK_PRESSURE data");
            };

            let fuel_tank_pressure = FuelTankPressure {
                fss_tnk_p1: fss_tnk_p.fuel_tank_pressure_1.0,
                fss_tnk_p2: fss_tnk_p.fuel_tank_pressure_2.0,
                fss_tnk_p_filtered: fss_tnk_p.dpr_pressure.0,
            };
            let pressurization_line_pressure = PressurizationLinePressure {
                prz_mnl_p: prz_mnl_p.0,
            };

            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(fuel_tank_pressure.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        trace!("[CAN Task] Sent pressure data: {:?}", fuel_tank_pressure);
                    }
                    Ok(Err(err)) => {
                        error!("[CAN Task] CAN TX error: {:?}", err);
                    }
                    Err(_) => {
                        error!("[CAN Task] CAN TX timed out after {} ms", TX_TIMEOUT_MS);
                    }
                }

                match with_timeout(
                    Duration::from_millis(TX_TIMEOUT_MS),
                    tx.transmit(pressurization_line_pressure.clone()),
                )
                .await
                {
                    Ok(Ok(_)) => {
                        trace!(
                            "[CAN Task] Sent pressure data: {:?}",
                            pressurization_line_pressure
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
    };

    // Valve States Task (≈5 Hz)
    let valve_states_task = async {
        let mut dpr_ctrl_watcher = STATE.dpr_control_loop.receiver().unwrap();
        let mut prz_vnt_watcher = STATE.pressurization_vent_control.receiver().unwrap();
        let mut fss_vnt_watcher = STATE.fuel_vent_control.receiver().unwrap();

        let mut data = FuelControlBoardValveStates {
            fss_dpr: Default::default(),
            prz_vnt_vlv: Default::default(),
            fss_vnt_vlv: Default::default(),
        };

        let mut ticker = Ticker::every(Duration::from_millis(
            1000 / CAN_VALVE_STATES_FREQ_HZ as u64,
        ));
        let mut changed = false;
        loop {
            if let Some(state) = dpr_ctrl_watcher.try_changed() {
                data.fss_dpr = state;
                changed = true;
            }
            if let Some(state) = prz_vnt_watcher.try_changed() {
                data.prz_vnt_vlv = state;
                changed = true;
            }
            if let Some(state) = fss_vnt_watcher.try_changed() {
                data.fss_vnt_vlv = state;
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
    };

    // Board Status Task (≈1 Hz)
    let board_status_task = async {
        let mut data = FuelControlBoardStatus {
            common: Default::default(),
            thermocouple_status: Online,
            pressure_bus: Online,
        };

        let start = Instant::now();
        let mut ticker = Ticker::every(Duration::from_millis(
            1000 / CAN_BOARD_STATUS_FREQ_HZ as u64,
        ));
        loop {
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
                        trace!("[CAN Task] sent Board Status: {:?}", data);
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
    };

    let build_information_task = async {
        let build_info = crate::build_info::BUILD_INFO.get();
        let build_info_msg = hermes_can::messages::debug_info::FuelControlBoardBuildInfo {
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
                    Ok(Ok(())) => trace!("Sent FuelControlBoardBuildInfo: {:?}", build_info_msg),
                    Ok(Err(err)) => error!("Error sending FuelControlBoardBuildInfo: {:?}", err),
                    Err(_) => error!(
                        "Timeout sending FuelControlBoardBuildInfo after {} ms",
                        TX_TIMEOUT_MS
                    ),
                }
            }

            ticker.next().await;
        }
    };

    let pressurization_info_task = async {
        let mut pressurization_info_watcher = STATE.pressurization_info.receiver().unwrap();

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
    };

    // Run all tasks concurrently
    join5(
        pressure_task,
        valve_states_task,
        board_status_task,
        build_information_task,
        pressurization_info_task,
    )
    .await;

    #[allow(unreachable_code)]
    loop {
        pending::<()>().await;
    }
}

/// Resets the system instantly and restarts the firmware.
fn reset_now() {
    cortex_m::peripheral::SCB::sys_reset();
}
