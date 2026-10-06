//! The only CAN transmitter: navigation state, board status and build info.
//! One task owns `CanTx` and sends one frame at a time.

use core::sync::atomic::Ordering;

use asteria_state_estimation::{GeodeticReference, NavigationState};
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
use crate::tasks::state_estimation::{height_msl_referenced, launch_site};

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
    let reference = launch_site();
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
                    send_state(&mut can_tx, &reference, &estimate).await;
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

async fn send_state(
    can_tx: &mut CanTx<'static>,
    reference: &GeodeticReference,
    state: &NavigationState,
) {
    use hermes_can::messages::sensor_data::{ImuData, OrientationData, PositionData, VelocityData};
    const RAD_TO_DEG: f32 = 180.0 / core::f32::consts::PI;

    let q = state.orientation_body_to_ned_wxyz;
    let [w, x, y, z] = q;
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

    let [velocity_north, velocity_east, velocity_down] = state.velocity_ned_mps;
    let [velocity_x, velocity_y, velocity_z] = rotate(conjugate(q), state.velocity_ned_mps);
    send(
        can_tx,
        VelocityData {
            velocity_x,
            velocity_y,
            velocity_z,
            velocity_north,
            velocity_east,
            velocity_down,
        },
    )
    .await;

    let rate_body_dps = state.angular_rate_body_rad_s.map(|rate| rate * RAD_TO_DEG);
    let [acceleration_x, acceleration_y, acceleration_z] = state.specific_force_body_mps2;
    let [angular_velocity_x, angular_velocity_y, angular_velocity_z] = rate_body_dps;
    let [acceleration_north, acceleration_east, acceleration_down] =
        rotate(q, state.specific_force_body_mps2);
    let [
        angular_velocity_north,
        angular_velocity_east,
        angular_velocity_down,
    ] = rotate(q, rate_body_dps);
    send(
        can_tx,
        ImuData {
            acceleration_x,
            acceleration_y,
            acceleration_z,
            angular_velocity_x,
            angular_velocity_y,
            angular_velocity_z,
            acceleration_north,
            acceleration_east,
            acceleration_down,
            angular_velocity_north,
            angular_velocity_east,
            angular_velocity_down,
        },
    )
    .await;

    let [north_std_m, east_std_m, down_std_m] = state.position_std_ned_m;
    if height_msl_referenced(down_std_m) {
        let position = reference.unproject(state.position_ned_m);
        send(
            can_tx,
            PositionData {
                location_latitude: position.latitude_deg,
                location_longitude: position.longitude_deg,
                location_hamsl: position.height_msl_m,
                horizontal_accuracy: north_std_m.max(east_std_m),
                vertical_accuracy: down_std_m,
            },
        )
        .await;
    }
}

fn conjugate([w, x, y, z]: [f32; 4]) -> [f32; 4] {
    [w, -x, -y, -z]
}

/// Rotates `v` by the unit quaternion `q` (scalar first).
fn rotate([w, x, y, z]: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    // v + 2w(u × v) + 2u × (u × v), with u the vector part of q.
    let cross = |a: [f32; 3], b: [f32; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let u = [x, y, z];
    let uv = cross(u, v);
    let uuv = cross(u, uv);
    core::array::from_fn(|i| v[i] + 2.0 * (w * uv[i] + uuv[i]))
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
