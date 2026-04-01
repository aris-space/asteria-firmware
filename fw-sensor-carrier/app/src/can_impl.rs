use crate::drivers::{environmental, inertial, magnetic_field, position_velocity, pressure};
use crate::sensors::{
    AtomicSensorStatus, BAROMETER_BUS_1_STATUS, BAROMETER_BUS_2_STATUS, DHT_BUS_1_STATUS,
    DHT_BUS_2_STATUS, GPS1_STATUS, GPS2_STATUS, IMU1_STATUS, IMU2_STATUS,
    MAGNETOMETER_BUS_1_STATUS, MAGNETOMETER_BUS_2_STATUS,
};
use core::sync::atomic::Ordering;
use embassy_executor::Spawner;
use embassy_stm32::can::enums::BusError;
use embassy_stm32::can::filter::{Action, FilterType, StandardFilter};
use embassy_stm32::can::frame::{self, FdFrame, Header};
use embassy_stm32::can::{Can, CanConfigurator, CanRx, CanTx, OperatingMode, RxPin, TxPin};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::{Peri, can};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Instant, Ticker, TimeoutError, with_timeout};
use embedded_can::Id;
use embedded_utils::fmt::*;
use hermes_can::messages::Message;
use hermes_can::messages::board_status::SensorStatus;
use hermes_can::{CanDecodeError, CanEncodeError, CanMessage, next_valid_length};
use nalgebra::Vector3;

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
                // to be less than 64.
                let dlc = next_valid_length(len).ok_or(CanError::Other)?;

                // zero-pad the payload to the next valid length.
                let payload = &buf[..dlc];

                // Barring embassy changes their implementation, this will never fail
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

    /* todo: unsure if this works, did not work in last year's project
    let mut config = can.config();
    config.global_filter = GlobalFilter::accept_all();
    can.set_config(config);
     */

    // TODO: Do not EVER forget to change this back to `NormalOperationMode` again after testing
    //  See :defeated-louis: meme (╯°□°)╯︵ ┻━┻
    can.start(OperatingMode::NormalOperationMode)
}

const THIS_BOARD_ID: hermes_can::messages::BoardId = hermes_can::messages::BoardId::SensorCarrier;

#[embassy_executor::task]
pub async fn can_rx_task(mut can_rx: CanRx<'static>) -> ! {
    loop {
        match can_rx.recv().await {
            Ok((msg, ts)) => {
                trace!("Received message {:?} on bus at {:?}", msg, ts);

                match msg {
                    Message::ResetAll(_) => {
                        warn!("Received ResetAll message, resetting Sensor Carrier");
                        reset_now();
                    }
                    Message::ResetSpecific(x) => {
                        if x.board_id == THIS_BOARD_ID {
                            warn!("Received ResetSpecific message, resetting Sensor Carrier");
                            reset_now();
                        }
                    }
                    _ => {
                        warn!("Received unknown CAN message: {:?}", msg);
                    }
                };
            }
            Err(err) => {
                error!("CAN RX error: {:?}", err);
            }
        }
    }
}

pub async fn spawn_can_tx_tasks(can_tx: CanTx<'static>, spawner: Spawner) {
    // Our transmission policy is as follows:
    // Send new data when available, but only if a minimum period has elapsed since the last
    // transmission. Otherwise, discard and wait for the next data.

    mod messages {
        pub use hermes_can::messages::sensor_data::*;
    }

    /// A module to group all CAN transmission configuration constants.
    mod task_config {
        use embassy_time::Duration;

        /// Common timeout for all CAN transmission attempts.
        pub const TX_TIMEOUT: Duration = Duration::from_millis(100);
        /// A factor to slightly reduce the minimum period between messages to avoid contention.
        const ALPHA: f32 = 0.2;

        /// Calculates the minimum period between transmissions based on a target frequency.
        const fn min_period(target_hz: f32) -> Duration {
            Duration::from_millis((1000.0 / (target_hz * (1.0 + ALPHA))) as u64)
        }

        // Configuration for each data transmission task.
        pub const PRESSURE_MIN_PERIOD: Duration = min_period(40.0);
        pub const ENVIRONMENTAL_MIN_PERIOD: Duration = min_period(1.0);
        pub const ORIENTATION_MIN_PERIOD: Duration = min_period(40.0);
        pub const MAGNETIC_FIELD_MIN_PERIOD: Duration = min_period(10.0);
        pub const POSITION_MIN_PERIOD: Duration = min_period(20.0);
        pub const VELOCITY_MIN_PERIOD: Duration = min_period(20.0);
        pub const INERTIAL_MIN_PERIOD: Duration = min_period(40.0);
        pub const STATUS_PERIOD: Duration = Duration::from_secs(1);
        pub const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);
        pub const NEW_DATA_TIMEOUT: Duration = Duration::from_secs(10);
    }

    static CAN_TX: OnceLock<Mutex<ThreadModeRawMutex, CanTx<'static>>> = OnceLock::new();

    CAN_TX
        .init(Mutex::new(can_tx))
        .ok()
        .expect("Failed to set CAN TX mutex");

    let can_tx = CAN_TX.get().await;

    // Pressure task (≈40 Hz)
    #[embassy_executor::task]
    async fn pressure_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut pressure_watch = pressure::PRESSURE_DRIVER_WATCH
            .receiver()
            .expect("failed to get pressure watch");

        loop {
            let pressure = loop {
                if let Ok(p) =
                    with_timeout(task_config::NEW_DATA_TIMEOUT, pressure_watch.changed()).await
                {
                    break p;
                }
                error!("Timeout waiting for pressure data");
            };

            let now = Instant::now();
            let can_data = messages::PressureData { pressure };

            if now - last_sent >= task_config::PRESSURE_MIN_PERIOD {
                let mut tx = can_tx.lock().await;
                match with_timeout(task_config::TX_TIMEOUT, tx.transmit(can_data.clone())).await {
                    Ok(Ok(())) => {
                        trace!("Sent pressure data: {:?}", can_data);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding pressure data: {:?}", can_data);
            }
        }
    }

    // Environmental task (≈1 Hz)
    #[embassy_executor::task]
    async fn environmental_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut environmental_watch = environmental::ENVIRONMENTAL_DRIVER_WATCH
            .receiver()
            .expect("failed to get environmental watch");

        loop {
            let env_data = loop {
                if let Ok(e) =
                    with_timeout(task_config::NEW_DATA_TIMEOUT, environmental_watch.changed()).await
                {
                    break e;
                }
                error!("Timeout waiting for environmental data");
            };

            let now = Instant::now();
            if now - last_sent >= task_config::ENVIRONMENTAL_MIN_PERIOD {
                let mut tx = can_tx.lock().await;
                match with_timeout(task_config::TX_TIMEOUT, tx.transmit(env_data.clone())).await {
                    Ok(Ok(())) => {
                        trace!("sent environmental data: {:?}", env_data);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding environmental data: {:?}", env_data);
            }
        }
    }

    // Orientation task (≈40 Hz)
    #[embassy_executor::task]
    async fn orientation_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut orientation_watch = inertial::ORIENTATION_WATCH
            .receiver()
            .expect("failed to get orientation watch");

        loop {
            let orientation = loop {
                if let Ok(o) =
                    with_timeout(task_config::NEW_DATA_TIMEOUT, orientation_watch.changed()).await
                {
                    break o;
                }
                error!("Timeout waiting for orientation data");
            };

            let can_data = messages::OrientationData {
                orientation_w: orientation.w,
                orientation_x: orientation.i,
                orientation_y: orientation.j,
                orientation_z: orientation.k,
            };

            let now = Instant::now();
            if now - last_sent >= task_config::ORIENTATION_MIN_PERIOD {
                let mut tx = can_tx.lock().await;
                match with_timeout(task_config::TX_TIMEOUT, tx.transmit(can_data.clone())).await {
                    Ok(Ok(())) => {
                        trace!("sent orientation data: {:?}", can_data);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding orientation data: {:?}", can_data);
            }
        }
    }

    // Magnetic field task (≈10 Hz)
    #[embassy_executor::task]
    async fn magnetic_field_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut orientation_anon = inertial::ORIENTATION_WATCH.anon_receiver();
        let mut magnetic_field_watch = magnetic_field::MAGNETIC_FIELD_WATCH
            .receiver()
            .expect("failed to get magnetic field watch");

        loop {
            let (_ts, field) = loop {
                if let Ok(x) = with_timeout(
                    task_config::NEW_DATA_TIMEOUT,
                    magnetic_field_watch.changed(),
                )
                .await
                {
                    break x;
                }
                error!("Timeout waiting for magnetic field data");
            };

            let orientation = orientation_anon.try_get().unwrap_or_default();

            // convert nT → µT
            let xyz = Vector3::new(
                field.x_nt() as f32 * 1e-3,
                field.y_nt() as f32 * 1e-3,
                field.z_nt() as f32 * 1e-3,
            );

            // passive rotation NED → body
            let ned = orientation.inverse() * xyz;

            let can_data = messages::MagnetometerData {
                magnetic_field_x: xyz.x,
                magnetic_field_y: xyz.y,
                magnetic_field_z: xyz.z,
                magnetic_field_north: ned.x,
                magnetic_field_east: ned.y,
                magnetic_field_down: ned.z,
            };

            let now = Instant::now();
            if now - last_sent >= task_config::MAGNETIC_FIELD_MIN_PERIOD {
                let mut tx = can_tx.lock().await;
                match with_timeout(task_config::TX_TIMEOUT, tx.transmit(can_data.clone())).await {
                    Ok(Ok(())) => {
                        trace!("sent magnetic field data: {:?}", can_data);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding magnetic field data: {:?}", can_data);
            }
        }
    }

    // Position task (≈20 Hz)
    #[embassy_executor::task]
    async fn position_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut position_watch = position_velocity::POSITION_WATCH
            .receiver()
            .expect("failed to get position watch");
        loop {
            let can_data = loop {
                if let Ok(p) =
                    with_timeout(task_config::NEW_DATA_TIMEOUT, position_watch.changed()).await
                {
                    break p;
                }
                error!("Timeout waiting for position data");
            };

            let now = Instant::now();
            if now - last_sent >= task_config::POSITION_MIN_PERIOD {
                let mut tx = can_tx.lock().await;
                match with_timeout(task_config::TX_TIMEOUT, tx.transmit(can_data.clone())).await {
                    Ok(Ok(())) => {
                        trace!("sent position data: {:?}", can_data);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding position data: {:?}", can_data);
            }
        }
    }

    // Velocity task (≈20 Hz)
    #[embassy_executor::task]
    async fn velocity_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut velocity_watch = position_velocity::VELOCITY_WATCH
            .receiver()
            .expect("failed to get velocity watch");
        loop {
            let can_data = loop {
                if let Ok(v) =
                    with_timeout(task_config::NEW_DATA_TIMEOUT, velocity_watch.changed()).await
                {
                    break v;
                }
                error!("Timeout waiting for velocity data");
            };

            let now = Instant::now();
            if now - last_sent >= task_config::VELOCITY_MIN_PERIOD {
                let mut tx = can_tx.lock().await;
                match with_timeout(task_config::TX_TIMEOUT, tx.transmit(can_data.clone())).await {
                    Ok(Ok(())) => {
                        trace!("sent velocity data: {:?}", can_data);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding velocity data: {:?}", can_data);
            }
        }
    }

    // Inertial task (≈40 Hz)
    #[embassy_executor::task]
    async fn inertial_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut inertial_watch = inertial::INERTIAL_WATCH
            .receiver()
            .expect("failed to get inertial watch");

        loop {
            let can_data = loop {
                if let Ok(i) =
                    with_timeout(task_config::NEW_DATA_TIMEOUT, inertial_watch.changed()).await
                {
                    break i;
                }
                error!("Timeout waiting for inertial data");
            };

            let now = Instant::now();

            if now - last_sent >= task_config::INERTIAL_MIN_PERIOD {
                let mut tx = can_tx.lock().await;
                match with_timeout(task_config::TX_TIMEOUT, tx.transmit(can_data.clone())).await {
                    Ok(Ok(())) => {
                        trace!("sent inertial data: {:?}", can_data);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding inertial data: {:?}", can_data);
            }
        }
    }

    // Status task (1 Hz)
    #[embassy_executor::task]
    async fn status_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        use hermes_can::messages::board_status::{
            SensorCarrierStatus, SensorsHealth, StatusCommonMessage,
        };
        let mut interval = Ticker::every(task_config::STATUS_PERIOD);

        loop {
            let convert = |status: &AtomicSensorStatus| match status.load(Ordering::Relaxed) {
                crate::sensors::SensorStatus::Inactive => SensorStatus::Offline,
                crate::sensors::SensorStatus::Active => SensorStatus::Online,
            };

            let micros_since_restart = Instant::now().as_micros();
            let full_status = SensorCarrierStatus {
                common: StatusCommonMessage {
                    errors: 0, // TODO sometime else
                    micros_since_restart,
                },
                sensor_health: SensorsHealth {
                    barometer_bus_1: convert(&BAROMETER_BUS_1_STATUS),
                    barometer_bus_2: convert(&BAROMETER_BUS_2_STATUS),
                    dht_bus_1: convert(&DHT_BUS_1_STATUS),
                    dht_bus_2: convert(&DHT_BUS_2_STATUS),
                    magnetometer_bus_1: convert(&MAGNETOMETER_BUS_1_STATUS),
                    magnetometer_bus_2: convert(&MAGNETOMETER_BUS_2_STATUS),
                    imu_1: convert(&IMU1_STATUS),
                    imu_2: convert(&IMU2_STATUS),
                    gps_1: convert(&GPS1_STATUS),
                    gps_2: convert(&GPS2_STATUS),
                },
            };

            // This scope guard ensures the lock is released before the next await point.
            {
                let mut can = can_tx.lock().await;
                if let Err(err) = can.transmit(full_status.clone()).await {
                    error!("CAN TX error: {:?}", err);
                } else {
                    trace!("sent status data: {:?}", full_status);
                }
            }

            interval.next().await;
        }
    }

    // Build Information task
    #[embassy_executor::task]
    async fn build_information(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
        let build_info = crate::build_info::BUILD_INFO.get();
        let build_info_msg = hermes_can::messages::debug_info::SensorCarrierBuildInfo {
            data: build_info.clone(),
        };

        let mut ticker = Ticker::every(task_config::BUILD_INFO_PERIOD);
        loop {
            // This scope is necessary to ensure the lock is released.
            {
                let mut tx = can_tx.lock().await;
                match with_timeout(task_config::TX_TIMEOUT, tx.transmit(build_info_msg.clone()))
                    .await
                {
                    Ok(Ok(())) => trace!("Sent SensorCarrierBuildInfo: {:?}", build_info_msg),
                    Ok(Err(err)) => error!("Error sending SensorCarrierBuildInfo: {:?}", err),
                    Err(_) => error!(
                        "Timeout sending SensorCarrierBuildInfo after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            }

            ticker.next().await;
        }
    }

    spawner
        .spawn(pressure_task(can_tx))
        .expect("Failed to spawn can pressure data task");

    spawner
        .spawn(environmental_task(can_tx))
        .expect("Failed to spawn can environmental data task");

    spawner
        .spawn(orientation_task(can_tx))
        .expect("Failed to spawn can orientation data task");

    spawner
        .spawn(magnetic_field_task(can_tx))
        .expect("Failed to spawn can magnetic field data task");

    spawner
        .spawn(position_task(can_tx))
        .expect("Failed to spawn can position data task");

    spawner
        .spawn(velocity_task(can_tx))
        .expect("Failed to spawn can velocity data task");

    spawner
        .spawn(inertial_task(can_tx))
        .expect("Failed to spawn can inertial data task");

    spawner
        .spawn(status_task(can_tx))
        .expect("Failed to spawn can status data task");

    spawner
        .spawn(build_information(can_tx))
        .expect("Failed to spawn can build information task");
}

/// Resets the system instantly and restarts the firmware.
fn reset_now() {
    cortex_m::peripheral::SCB::sys_reset();
}
