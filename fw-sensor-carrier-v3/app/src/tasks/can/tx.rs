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

use super::CanTransmitter;
use crate::sensors::{
    AtomicSensorStatus, BARO_STATUS, DHT_STATUS, GNSS_STATUS, IMU_STATUS, MAG_STATUS, SensorStatus,
};
use crate::signals;

const TX_TIMEOUT: Duration = Duration::from_millis(100);
const NEW_DATA_TIMEOUT: Duration = Duration::from_secs(10);
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);

// Just under 20 Hz, so the 20 Hz estimate is never throttled by jitter.
const STATE_MIN_PERIOD: Duration = Duration::from_millis(41);

static CAN_TX: OnceLock<Mutex<ThreadModeRawMutex, CanTx<'static>>> = OnceLock::new();

pub fn spawn(can_tx: CanTx<'static>, spawner: Spawner) {
    CAN_TX
        .init(Mutex::new(can_tx))
        .ok()
        .expect("CAN TX init twice");
    let can_tx = CAN_TX.try_get().expect("CAN TX not yet initialized");

    spawner.spawn(state_task(can_tx).expect("spawn can state"));
    spawner.spawn(status_task(can_tx).expect("spawn can status"));
    spawner.spawn(build_information_task(can_tx).expect("spawn can build info"));
}

async fn send<M: CanMessage + Clone + defmt::Format>(
    can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>,
    msg: M,
) {
    let mut tx = can_tx.lock().await;
    match with_timeout(TX_TIMEOUT, tx.transmit(msg.clone())).await {
        Ok(Ok(())) => trace!("CAN: sent {:?}", msg),
        Ok(Err(err)) => error!("CAN: TX error: {:?}", err),
        Err(_) => error!("CAN: TX timed out after {} ms", TX_TIMEOUT.as_millis()),
    }
}

#[embassy_executor::task]
async fn state_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::{OrientationData, VerticalStateData};
    let mut rx = signals::STATE_ESTIMATE_WATCH
        .receiver()
        .expect("CAN: state estimate receiver available");
    let mut last_sent = Instant::now();
    loop {
        let Ok(estimate) = with_timeout(NEW_DATA_TIMEOUT, rx.changed()).await else {
            error!("CAN: no state estimate for 10 s");
            continue;
        };
        let now = Instant::now();
        if now - last_sent < STATE_MIN_PERIOD {
            continue;
        }
        last_sent = now;
        let [w, x, y, z] = estimate.orientation_body_to_ned_wxyz;
        // The CAN contract uses the inverse (NED-to-body) quaternion.
        send(
            can_tx,
            OrientationData {
                orientation_w: w,
                orientation_x: -x,
                orientation_y: -y,
                orientation_z: -z,
            },
        )
        .await;
        if estimate.msl_ready {
            send(
                can_tx,
                VerticalStateData {
                    height_m: estimate.height_msl_m,
                    velocity_mps: estimate.velocity_mps,
                    height_std_m: estimate.height_std_m,
                    velocity_std_mps: estimate.velocity_std_mps,
                    selected_imu: estimate.selected_imu.index() as u8,
                    redundancy_ready: estimate.redundancy_ready,
                },
            )
            .await;
        }
    }
}

#[embassy_executor::task]
async fn status_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::board_status::{
        SensorCarrierStatus, SensorsHealth, StatusCommonMessage,
    };
    let mut ticker = Ticker::every(STATUS_PERIOD);

    fn convert(s: &AtomicSensorStatus) -> CanSensorStatus {
        match s.load(Ordering::Relaxed) {
            SensorStatus::Inactive | SensorStatus::Disabled => CanSensorStatus::Offline,
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
                barometer_bus_1: convert(&BARO_STATUS[0]),
                barometer_bus_2: convert(&BARO_STATUS[1]),
                dht_bus_1: convert(&DHT_STATUS[0]),
                dht_bus_2: convert(&DHT_STATUS[1]),
                magnetometer_bus_1: convert(&MAG_STATUS[0]),
                magnetometer_bus_2: convert(&MAG_STATUS[1]),
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
