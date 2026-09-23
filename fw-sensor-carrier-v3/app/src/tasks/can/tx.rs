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

const VERTICAL_MIN_PERIOD: Duration = min_period(20.0);

static CAN_TX: OnceLock<Mutex<ThreadModeRawMutex, CanTx<'static>>> = OnceLock::new();

pub fn spawn_tx_tasks(can_tx: CanTx<'static>, spawner: Spawner) {
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
async fn state_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::{OrientationData, VerticalStateData};
    watch_loop!(
        signals::STATE_ESTIMATE_WATCH,
        VERTICAL_MIN_PERIOD,
        |estimate| {
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
    );
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
