use core::sync::atomic::Ordering;

use defmt::{error, trace};
use embassy_executor::Spawner;
use embassy_stm32::can::CanTx;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use hermes_can::CanMessage;
use hermes_can::messages::board_status::SensorStatus as CanSensorStatus;
use nalgebra::Vector3;

use super::CanTransmitter;
use crate::sensors::{
    AtomicSensorStatus, BAROMETER_STATUS, DHT_STATUS, GNSS_STATUS, IMU_STATUS, MAGNETOMETER_STATUS,
    SensorStatus,
};
use crate::signals;

const TX_TIMEOUT: Duration = Duration::from_millis(100);
const NEW_DATA_TIMEOUT: Duration = Duration::from_secs(10);
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);

/// Cadence cap per derived signal. `min_period(hz)` is slightly under the
/// natural source rate so a steady producer slips through.
const fn min_period(target_hz: f32) -> Duration {
    const ALPHA: f32 = 0.2;
    Duration::from_millis((1000.0 / (target_hz * (1.0 + ALPHA))) as u64)
}

const PRESSURE_MIN_PERIOD: Duration = min_period(40.0);
const ENVIRONMENT_MIN_PERIOD: Duration = min_period(1.0);
const ORIENTATION_MIN_PERIOD: Duration = min_period(40.0);
const MAG_MIN_PERIOD: Duration = min_period(10.0);
const POSITION_MIN_PERIOD: Duration = min_period(20.0);
const VELOCITY_MIN_PERIOD: Duration = min_period(20.0);
const INERTIAL_MIN_PERIOD: Duration = min_period(40.0);

static CAN_TX: OnceLock<Mutex<ThreadModeRawMutex, CanTx<'static>>> = OnceLock::new();

pub fn spawn_tx_tasks(can_tx: CanTx<'static>, spawner: Spawner) {
    CAN_TX
        .init(Mutex::new(can_tx))
        .ok()
        .expect("CAN TX init twice");
    let can_tx = CAN_TX.try_get().expect("CAN TX not yet initialized");

    spawner.spawn(pressure_task(can_tx).expect("spawn can pressure"));
    spawner.spawn(environment_task(can_tx).expect("spawn can env"));
    spawner.spawn(orientation_task(can_tx).expect("spawn can orientation"));
    spawner.spawn(mag_task(can_tx).expect("spawn can mag"));
    spawner.spawn(position_task(can_tx).expect("spawn can position"));
    spawner.spawn(velocity_task(can_tx).expect("spawn can velocity"));
    spawner.spawn(inertial_task(can_tx).expect("spawn can inertial"));
    spawner.spawn(status_task(can_tx).expect("spawn can status"));
    spawner.spawn(build_information_task(can_tx).expect("spawn can build info"));
}

async fn send<M: CanMessage + Clone + defmt::Format>(
    can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>,
    msg: M,
) {
    let mut tx = can_tx.lock().await;
    match with_timeout(TX_TIMEOUT, tx.transmit(msg.clone())).await {
        Ok(Ok(())) => trace!("CAN sent: {:?}", msg),
        Ok(Err(err)) => error!("CAN TX error: {:?}", err),
        Err(_) => error!("CAN TX timed out after {} ms", TX_TIMEOUT.as_millis()),
    }
}

/// Drive a per-signal TX task: wait for the watch to change, drop the value if
/// the rate cap hasn't elapsed, otherwise send.
macro_rules! watch_loop {
    ($watch:expr, $min_period:expr, |$val:ident| $body:block) => {
        let mut last_sent = Instant::now();
        let mut rx = $watch
            .receiver()
            .expect("CAN: failed to create watch receiver");
        loop {
            let $val = loop {
                match with_timeout(NEW_DATA_TIMEOUT, rx.changed()).await {
                    Ok(v) => break v,
                    Err(_) => error!("Timeout waiting for CAN watch data"),
                }
            };
            let now = Instant::now();
            if now - last_sent >= $min_period {
                $body
                last_sent = now;
            }
        }
    };
}

#[embassy_executor::task]
async fn pressure_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::PressureData;
    watch_loop!(signals::PRESSURE_WATCH, PRESSURE_MIN_PERIOD, |pressure| {
        send(
            can_tx,
            PressureData {
                pressure: pressure.mbar,
            },
        )
        .await;
    });
}

#[embassy_executor::task]
async fn environment_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::EnvironmentalData;
    watch_loop!(signals::ENVIRONMENT_WATCH, ENVIRONMENT_MIN_PERIOD, |env| {
        let msg = EnvironmentalData {
            temperature: env.temperature_c,
            humidity: env.humidity_rh,
            pressure: env.pressure_mbar,
        };
        send(can_tx, msg).await;
    });
}

#[embassy_executor::task]
async fn orientation_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::OrientationData;
    watch_loop!(signals::ORIENTATION_WATCH, ORIENTATION_MIN_PERIOD, |o| {
        let msg = OrientationData {
            orientation_w: o.q.w,
            orientation_x: o.q.i,
            orientation_y: o.q.j,
            orientation_z: o.q.k,
        };
        send(can_tx, msg).await;
    });
}

#[embassy_executor::task]
async fn mag_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::MagnetometerData;
    let mut orientation_rx = signals::ORIENTATION_WATCH.anon_receiver();
    watch_loop!(signals::MAG_WATCH, MAG_MIN_PERIOD, |field| {
        let orientation = orientation_rx.try_get().map(|o| o.q).unwrap_or_default();
        // nT -> uT. xyz is body-frame; orientation is body -> NED.
        let xyz = Vector3::new(field.x * 1e-3, field.y * 1e-3, field.z * 1e-3);
        let ned = orientation * xyz;
        let msg = MagnetometerData {
            magnetic_field_x: xyz.x,
            magnetic_field_y: xyz.y,
            magnetic_field_z: xyz.z,
            magnetic_field_north: ned.x,
            magnetic_field_east: ned.y,
            magnetic_field_down: ned.z,
        };
        send(can_tx, msg).await;
    });
}

#[embassy_executor::task]
async fn position_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::PositionData;
    watch_loop!(signals::POSITION_WATCH, POSITION_MIN_PERIOD, |pos| {
        let msg = PositionData {
            location_latitude: pos.lat_deg,
            location_longitude: pos.lon_deg,
            location_hamsl: pos.height_msl_m,
            horizontal_accuracy: pos.horizontal_accuracy_m,
            vertical_accuracy: pos.vertical_accuracy_m,
        };
        send(can_tx, msg).await;
    });
}

#[embassy_executor::task]
async fn velocity_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::VelocityData;
    watch_loop!(signals::VELOCITY_WATCH, VELOCITY_MIN_PERIOD, |vel| {
        let msg = VelocityData {
            velocity_x: vel.body_x,
            velocity_y: vel.body_y,
            velocity_z: vel.body_z,
            velocity_north: vel.ned_north,
            velocity_east: vel.ned_east,
            velocity_down: vel.ned_down,
        };
        send(can_tx, msg).await;
    });
}

#[embassy_executor::task]
async fn inertial_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::ImuData;
    watch_loop!(signals::INERTIAL_WATCH, INERTIAL_MIN_PERIOD, |inertial| {
        let msg = ImuData {
            acceleration_x: inertial.body_accel_x,
            acceleration_y: inertial.body_accel_y,
            acceleration_z: inertial.body_accel_z,
            angular_velocity_x: inertial.body_gyro_x,
            angular_velocity_y: inertial.body_gyro_y,
            angular_velocity_z: inertial.body_gyro_z,
            acceleration_north: inertial.ned_accel_north,
            acceleration_east: inertial.ned_accel_east,
            acceleration_down: inertial.ned_accel_down,
            angular_velocity_north: inertial.ned_gyro_north,
            angular_velocity_east: inertial.ned_gyro_east,
            angular_velocity_down: inertial.ned_gyro_down,
        };
        send(can_tx, msg).await;
    });
}

#[embassy_executor::task]
async fn status_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::board_status::{
        SensorCarrierStatus, SensorsHealth, StatusCommonMessage,
    };
    let mut ticker = Ticker::every(STATUS_PERIOD);

    fn convert(s: &AtomicSensorStatus) -> CanSensorStatus {
        match s.load(Ordering::Relaxed) {
            SensorStatus::Inactive => CanSensorStatus::Offline,
            SensorStatus::Active => CanSensorStatus::Online,
        }
    }

    loop {
        let micros_since_restart = Instant::now().as_micros();
        let msg = SensorCarrierStatus {
            common: StatusCommonMessage {
                errors: 0,
                micros_since_restart,
            },
            sensor_health: SensorsHealth {
                barometer_bus_1: convert(&BAROMETER_STATUS[0]),
                barometer_bus_2: convert(&BAROMETER_STATUS[1]),
                dht_bus_1: convert(&DHT_STATUS[0]),
                dht_bus_2: convert(&DHT_STATUS[1]),
                magnetometer_bus_1: convert(&MAGNETOMETER_STATUS[0]),
                magnetometer_bus_2: convert(&MAGNETOMETER_STATUS[1]),
                imu_1: convert(&IMU_STATUS[0]),
                imu_2: convert(&IMU_STATUS[1]),
                gps_1: convert(&GNSS_STATUS[0]),
                gps_2: convert(&GNSS_STATUS[1]),
            },
        };
        send(can_tx, msg).await;
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn build_information_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::debug_info::{BuildInformationCommon, SensorCarrierBuildInfo};
    let info = crate::build_info::BUILD_INFO.get();
    let msg = SensorCarrierBuildInfo {
        data: BuildInformationCommon {
            unix_timestamp: info.unix_timestamp,
            author_initials: info.author_initials,
            is_release: info.is_release,
            debug_defmt_rtt: info.debug_defmt_rtt,
            commit_hash: info.commit_hash,
            is_git_dirty: info.is_git_dirty,
            can_semver: [
                hermes_can::VERSION_MAJOR,
                hermes_can::VERSION_MINOR,
                hermes_can::VERSION_PATCH,
            ],
        },
    };
    let mut ticker = Ticker::every(BUILD_INFO_PERIOD);
    loop {
        send(can_tx, msg.clone()).await;
        ticker.next().await;
    }
}
