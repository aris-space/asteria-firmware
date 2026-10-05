//! The only CAN transmitter: state estimates, board status and build info.
//! One task owns `CanTx` and sends one frame at a time.

use core::sync::atomic::Ordering;

use defmt::{error, trace};
use embassy_futures::select::{Either3, select3};
use embassy_stm32::can::CanTx;
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use hermes_can::CanMessage;
use hermes_can::messages::board_status::SensorStatus as CanSensorStatus;

use super::CanTransmitter;
use crate::sensors::{
    AtomicSensorStatus, BARO_STATUS, DHT_STATUS, GNSS_STATUS, IMU_STATUS, MAG_STATUS, SensorStatus,
};
use crate::signals;
use crate::types::StateEstimate;

const TX_TIMEOUT: Duration = Duration::from_millis(100);
const NEW_DATA_TIMEOUT: Duration = Duration::from_secs(10);
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);

// Just under 20 Hz, so the 20 Hz estimate is never throttled by jitter.
const STATE_MIN_PERIOD: Duration = Duration::from_millis(41);

#[embassy_executor::task]
pub async fn task(mut can_tx: CanTx<'static>) -> ! {
    let mut estimates = signals::STATE_ESTIMATE_WATCH
        .receiver()
        .expect("CAN: state estimate receiver available");
    let mut status = Ticker::every(STATUS_PERIOD);
    let mut build_info = Ticker::every(BUILD_INFO_PERIOD);
    let mut last_estimate = Instant::now();
    let mut last_state_sent = Instant::now();
    let mut estimate_missing = false;
    loop {
        match select3(estimates.changed(), status.next(), build_info.next()).await {
            Either3::First(estimate) => {
                let now = Instant::now();
                last_estimate = now;
                estimate_missing = false;
                if now - last_state_sent >= STATE_MIN_PERIOD {
                    last_state_sent = now;
                    send_state(&mut can_tx, &estimate).await;
                }
            }
            Either3::Second(()) => {
                if !estimate_missing && last_estimate.elapsed() >= NEW_DATA_TIMEOUT {
                    error!("CAN: no state estimate for 10 s");
                    estimate_missing = true;
                }
                send(&mut can_tx, status_message()).await;
            }
            Either3::Third(()) => {
                use hermes_can::messages::debug_info::SensorCarrierBuildInfo;
                let msg = SensorCarrierBuildInfo {
                    data: crate::built::can_build_information(),
                };
                send(&mut can_tx, msg).await;
            }
        }
    }
}

async fn send<M: CanMessage + Clone + defmt::Format>(can_tx: &mut CanTx<'static>, msg: M) {
    match with_timeout(TX_TIMEOUT, can_tx.transmit(msg.clone())).await {
        Ok(Ok(())) => trace!("CAN: sent {:?}", msg),
        Ok(Err(err)) => error!("CAN: TX error: {:?}", err),
        Err(_) => error!("CAN: TX timed out after {} ms", TX_TIMEOUT.as_millis()),
    }
}

async fn send_state(can_tx: &mut CanTx<'static>, estimate: &StateEstimate) {
    use hermes_can::messages::sensor_data::{OrientationData, VerticalStateData};
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

fn status_message() -> hermes_can::messages::board_status::SensorCarrierStatus {
    use hermes_can::messages::board_status::{
        SensorCarrierStatus, SensorsHealth, StatusCommonMessage,
    };

    fn convert(s: &AtomicSensorStatus) -> CanSensorStatus {
        match s.load(Ordering::Relaxed) {
            SensorStatus::Inactive => CanSensorStatus::Offline,
            SensorStatus::Active => CanSensorStatus::Online,
        }
    }

    SensorCarrierStatus {
        common: StatusCommonMessage {
            errors: 0,
            micros_since_restart: Instant::now().as_micros(),
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
    }
}
