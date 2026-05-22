use crate::drivers::{environmental, inertial, magnetic_field, position_velocity, pressure};
use crate::sensors::{
    AtomicSensorStatus, BAROMETER_BUS_1_STATUS, BAROMETER_BUS_2_STATUS, DHT_BUS_1_STATUS,
    DHT_BUS_2_STATUS, GPS1_STATUS, GPS2_STATUS, IMU1_STATUS, IMU2_STATUS,
    MAGNETOMETER_BUS_1_STATUS, MAGNETOMETER_BUS_2_STATUS,
};
use can_utils::rxtx::{TypedCanReceive, TypedCanTransmit};
use core::sync::atomic::Ordering;
use data_core::can::{hal::CanDecode, sparse_decodable_can_message};
use datatypes::status::{BoardId, SensorStatus, StatusCommonMessage};
use datatypes::units::HPa;
use dp_sensor_carrier::{
    EnvironmentalData, ImuData, MagnetometerData, Message, OrientationData, PositionData,
    SensorCarrierStatus, SensorsHealth, VelocityData,
};
use embassy_executor::Spawner;
use embassy_stm32::can::{CanRx, CanTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Instant, Ticker, with_timeout};
use embedded_utils::fmt::*;
use nalgebra::Vector3;

const THIS_BOARD_ID: BoardId = BoardId::SensorCarrier;

// Messages received on the bus that the Sensor Carrier cares about.
sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
    }
}

// This will fail to compile if the number of enabled ids exceeds the number of filters.
const __ASSERT_LEN_OK: () = {
    const FILTER_COUNT: usize = 28;
    if ReceivedMessage::SUPPORTED_IDS.len() >= FILTER_COUNT - 1 {
        core::panic!("Too many receiving can ids");
    }
};

#[embassy_executor::task]
pub async fn can_rx_task(mut can_rx: CanRx<'static>) -> ! {
    loop {
        match can_rx.recv::<ReceivedMessage>().await {
            Ok(msg) => match msg {
                ReceivedMessage::ResetAll(_) => {
                    warn!("Received ResetAll message, resetting Sensor Carrier");
                    reset_now();
                }
                ReceivedMessage::ResetSpecific(board_id) => {
                    if board_id == THIS_BOARD_ID {
                        warn!("Received ResetSpecific message, resetting Sensor Carrier");
                        reset_now();
                    }
                }
            },
            Err(err) => {
                error!("CAN RX error: {:?}", err);
            }
        }
    }
}

pub async fn spawn_can_tx_tasks(
    can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>,
    spawner: Spawner,
) {
    // Our transmission policy is as follows:
    // Send new data when available, but only if a minimum period has elapsed since the last
    // transmission. Otherwise, discard and wait for the next data.

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

    // Pressure task (≈40 Hz)
    #[embassy_executor::task]
    async fn pressure_task(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
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
            let payload = HPa(pressure);

            if now - last_sent >= task_config::PRESSURE_MIN_PERIOD {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    task_config::TX_TIMEOUT,
                    tx.transmit(Message::PressureData(payload)),
                )
                .await
                {
                    Ok(Ok(())) => {
                        trace!("Sent pressure data: {:?}", payload);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding pressure data: {:?}", payload);
            }
        }
    }

    // Environmental task (≈1 Hz)
    #[embassy_executor::task]
    async fn environmental_task(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut environmental_watch = environmental::ENVIRONMENTAL_DRIVER_WATCH
            .receiver()
            .expect("failed to get environmental watch");

        loop {
            let env_data: EnvironmentalData = loop {
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
                match with_timeout(
                    task_config::TX_TIMEOUT,
                    tx.transmit(Message::EnvironmentalData(env_data.clone())),
                )
                .await
                {
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
    async fn orientation_task(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
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

            let payload = OrientationData {
                orientation_w: orientation.w,
                orientation_x: orientation.i,
                orientation_y: orientation.j,
                orientation_z: orientation.k,
            };

            let now = Instant::now();
            if now - last_sent >= task_config::ORIENTATION_MIN_PERIOD {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    task_config::TX_TIMEOUT,
                    tx.transmit(Message::OrientationData(payload.clone())),
                )
                .await
                {
                    Ok(Ok(())) => {
                        trace!("sent orientation data: {:?}", payload);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding orientation data: {:?}", payload);
            }
        }
    }

    // Magnetic field task (≈10 Hz)
    #[embassy_executor::task]
    async fn magnetic_field_task(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
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

            let payload = MagnetometerData {
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
                match with_timeout(
                    task_config::TX_TIMEOUT,
                    tx.transmit(Message::MagnetometerData(payload.clone())),
                )
                .await
                {
                    Ok(Ok(())) => {
                        trace!("sent magnetic field data: {:?}", payload);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding magnetic field data: {:?}", payload);
            }
        }
    }

    // Position task (≈20 Hz)
    #[embassy_executor::task]
    async fn position_task(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut position_watch = position_velocity::POSITION_WATCH
            .receiver()
            .expect("failed to get position watch");
        loop {
            let payload: PositionData = loop {
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
                match with_timeout(
                    task_config::TX_TIMEOUT,
                    tx.transmit(Message::PositionData(payload.clone())),
                )
                .await
                {
                    Ok(Ok(())) => {
                        trace!("sent position data: {:?}", payload);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding position data: {:?}", payload);
            }
        }
    }

    // Velocity task (≈20 Hz)
    #[embassy_executor::task]
    async fn velocity_task(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut velocity_watch = position_velocity::VELOCITY_WATCH
            .receiver()
            .expect("failed to get velocity watch");
        loop {
            let payload: VelocityData = loop {
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
                match with_timeout(
                    task_config::TX_TIMEOUT,
                    tx.transmit(Message::VelocityData(payload.clone())),
                )
                .await
                {
                    Ok(Ok(())) => {
                        trace!("sent velocity data: {:?}", payload);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding velocity data: {:?}", payload);
            }
        }
    }

    // Inertial task (≈40 Hz)
    #[embassy_executor::task]
    async fn inertial_task(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
        let mut last_sent = Instant::now();
        let mut inertial_watch = inertial::INERTIAL_WATCH
            .receiver()
            .expect("failed to get inertial watch");

        loop {
            let payload: ImuData = loop {
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
                match with_timeout(
                    task_config::TX_TIMEOUT,
                    tx.transmit(Message::ImuData(payload.clone())),
                )
                .await
                {
                    Ok(Ok(())) => {
                        trace!("sent inertial data: {:?}", payload);
                        last_sent = now;
                    }
                    Ok(Err(err)) => error!("CAN TX error: {:?}", err),
                    Err(_) => error!(
                        "CAN TX timed out after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            } else {
                trace!("Discarding inertial data: {:?}", payload);
            }
        }
    }

    // Status task (1 Hz)
    #[embassy_executor::task]
    async fn status_task(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
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

            {
                let mut tx = can_tx.lock().await;
                if let Err(err) = tx.transmit(Message::BoardStatus(full_status.clone())).await {
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
    async fn build_information(can_tx: &'static Mutex<CriticalSectionRawMutex, CanTx<'static>>) {
        let build_info = crate::build_info::BUILD_INFO.get();

        let mut ticker = Ticker::every(task_config::BUILD_INFO_PERIOD);
        loop {
            {
                let mut tx = can_tx.lock().await;
                match with_timeout(
                    task_config::TX_TIMEOUT,
                    tx.transmit(Message::BuildInfo(build_info.clone())),
                )
                .await
                {
                    Ok(Ok(())) => trace!("Sent build info: {:?}", build_info),
                    Ok(Err(err)) => error!("Error sending build info: {:?}", err),
                    Err(_) => error!(
                        "Timeout sending build info after {} ms",
                        task_config::TX_TIMEOUT.as_millis()
                    ),
                }
            }

            ticker.next().await;
        }
    }

    spawner.spawn(pressure_task(can_tx).expect("Failed to spawn can pressure data task"));

    spawner.spawn(environmental_task(can_tx).expect("Failed to spawn can environmental data task"));

    spawner.spawn(orientation_task(can_tx).expect("Failed to spawn can orientation data task"));

    spawner
        .spawn(magnetic_field_task(can_tx).expect("Failed to spawn can magnetic field data task"));

    spawner.spawn(position_task(can_tx).expect("Failed to spawn can position data task"));

    spawner.spawn(velocity_task(can_tx).expect("Failed to spawn can velocity data task"));

    spawner.spawn(inertial_task(can_tx).expect("Failed to spawn can inertial data task"));

    spawner.spawn(status_task(can_tx).expect("Failed to spawn can status data task"));

    spawner.spawn(build_information(can_tx).expect("Failed to spawn can build information task"));
}

/// Resets the system instantly and restarts the firmware.
fn reset_now() -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}
