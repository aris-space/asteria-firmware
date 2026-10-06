//! CAN transmission of every `dp-sensor-carrier` message.
//!
//! Each message has its own task that builds it at the message's rate and
//! hands it to the CAN peripheral's TX buffer without waiting; the driver's
//! interrupt moves frames from there into the hardware. With a dead bus the
//! buffer fills and new frames are dropped.

use core::sync::atomic::Ordering;

use asteria_sef_core::NavigationState;
use data_core::can::hal::CanEncode as _;
use datatypes::status::{SensorStatus as CanSensorStatus, StatusCommonMessage};
use datatypes::units::{Celsius, HPa};
use defmt::{error, trace};
use dp_sensor_carrier::{
    EnvironmentalData, ImuData, MagnetometerData, Message, OrientationData, PositionData,
    SensorCarrierStatus, SensorsHealth, VelocityData,
};
use embassy_stm32::can::BufferedFdCanSender;
use embassy_stm32::can::frame::{FdFrame, Header};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::sensors::{
    AtomicSensorStatus, BARO_STATUS, DHT_STATUS, GNSS_STATUS, IMU_STATUS, MAG_STATUS, SensorStatus,
};
use crate::signals;
use crate::tasks::state_estimation::{height_msl_referenced, launch_site};

const ORIENTATION_PERIOD: Duration = Duration::from_hz(40);
const IMU_PERIOD: Duration = Duration::from_hz(40);
const POSITION_PERIOD: Duration = Duration::from_hz(20);
const VELOCITY_PERIOD: Duration = Duration::from_hz(20);
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);

// The estimator publishes with every IMU sample. Older state means it stopped.
const STATE_MAX_AGE: Duration = Duration::from_millis(100);

/// Encodes `msg` and hands it to the CAN peripheral; a full TX buffer means
/// the bus takes no frames, and the frame is dropped.
fn send(can: &mut BufferedFdCanSender, msg: Message) {
    let mut payload = [0u8; 64];
    let Ok((id, len)) = msg.encode_into(&mut payload) else {
        error!("CAN: a message failed to encode");
        return;
    };
    let Ok(frame) = FdFrame::new(
        Header::new(id.into(), len, false),
        &payload[..usize::from(len)],
    ) else {
        error!("CAN: a frame failed to build");
        return;
    };
    if can.try_write(frame).is_err() {
        trace!("CAN: TX buffer full, frame dropped");
    }
}

/// Sends the message `build` returns every `period`, if it returns one.
async fn every(
    mut can: BufferedFdCanSender,
    period: Duration,
    mut build: impl FnMut() -> Option<Message>,
) -> ! {
    let mut ticker = Ticker::every(period);
    loop {
        ticker.next().await;
        if let Some(msg) = build() {
            send(&mut can, msg);
        }
    }
}

/// Sends the message `build` makes from every new value of `watch`, if it makes one.
async fn on_change<T: Clone>(
    mut can: BufferedFdCanSender,
    watch: &'static Watch<CriticalSectionRawMutex, T, 1>,
    mut build: impl FnMut(T) -> Option<Message>,
) -> ! {
    let mut receiver = watch.receiver().expect("CAN: watch receiver slot");
    loop {
        if let Some(msg) = build(receiver.changed().await) {
            send(&mut can, msg);
        }
    }
}

#[embassy_executor::task]
pub async fn orientation(can: BufferedFdCanSender) -> ! {
    every(can, ORIENTATION_PERIOD, || {
        // The CAN contract uses the NED-to-body rotation.
        let ned_to_body = body_to_ned(&fresh_state()?).inverse();
        Some(Message::OrientationData(OrientationData {
            orientation_w: ned_to_body.w,
            orientation_x: ned_to_body.i,
            orientation_y: ned_to_body.j,
            orientation_z: ned_to_body.k,
        }))
    })
    .await
}

#[embassy_executor::task]
pub async fn imu(can: BufferedFdCanSender) -> ! {
    every(can, IMU_PERIOD, || {
        let state = fresh_state()?;
        let body_to_ned = body_to_ned(&state);
        let acceleration = Vector3::from(state.specific_force_body_mps2);
        let rate_dps = Vector3::from(state.angular_rate_body_rad_s).map(f32::to_degrees);
        let acceleration_ned = body_to_ned * acceleration;
        let rate_ned_dps = body_to_ned * rate_dps;
        Some(Message::ImuData(ImuData {
            acceleration_x: acceleration.x,
            acceleration_y: acceleration.y,
            acceleration_z: acceleration.z,
            angular_velocity_x: rate_dps.x,
            angular_velocity_y: rate_dps.y,
            angular_velocity_z: rate_dps.z,
            acceleration_north: acceleration_ned.x,
            acceleration_east: acceleration_ned.y,
            acceleration_down: acceleration_ned.z,
            angular_velocity_north: rate_ned_dps.x,
            angular_velocity_east: rate_ned_dps.y,
            angular_velocity_down: rate_ned_dps.z,
        }))
    })
    .await
}

#[embassy_executor::task]
pub async fn velocity(can: BufferedFdCanSender) -> ! {
    every(can, VELOCITY_PERIOD, || {
        let state = fresh_state()?;
        let ned = Vector3::from(state.velocity_ned_mps);
        let body = body_to_ned(&state).inverse() * ned;
        Some(Message::VelocityData(VelocityData {
            velocity_x: body.x,
            velocity_y: body.y,
            velocity_z: body.z,
            velocity_north: ned.x,
            velocity_east: ned.y,
            velocity_down: ned.z,
        }))
    })
    .await
}

/// Sent once GNSS has referenced the height to MSL.
#[embassy_executor::task]
pub async fn position(can: BufferedFdCanSender) -> ! {
    let reference = launch_site();
    every(can, POSITION_PERIOD, || {
        let state = fresh_state()?;
        let [north_std_m, east_std_m, down_std_m] = state.position_std_ned_m;
        if !height_msl_referenced(down_std_m) {
            return None;
        }
        let position = reference.unproject(state.position_ned_m);
        Some(Message::PositionData(PositionData {
            location_latitude: position.latitude_deg,
            location_longitude: position.longitude_deg,
            location_hamsl: position.height_msl_m,
            horizontal_accuracy: north_std_m.max(east_std_m),
            vertical_accuracy: down_std_m,
        }))
    })
    .await
}

/// Every reading of the selected barometer.
#[embassy_executor::task]
pub async fn pressure(can: BufferedFdCanSender) -> ! {
    on_change(can, &signals::SELECTED_BARO, |reading| {
        Some(Message::PressureData(HPa(reading.cal.pressure_mbar)))
    })
    .await
}

/// Every reading of the selected magnetometer, in body and NED axes.
#[embassy_executor::task]
pub async fn magnetometer(can: BufferedFdCanSender) -> ! {
    const NT_TO_UT: f32 = 1e-3;
    on_change(can, &signals::SELECTED_MAG, |reading| {
        let body = Vector3::new(reading.cal.x, reading.cal.y, reading.cal.z) * NT_TO_UT;
        let ned = body_to_ned(&fresh_state()?) * body;
        Some(Message::MagnetometerData(MagnetometerData {
            magnetic_field_x: body.x,
            magnetic_field_y: body.y,
            magnetic_field_z: body.z,
            magnetic_field_north: ned.x,
            magnetic_field_east: ned.y,
            magnetic_field_down: ned.z,
        }))
    })
    .await
}

/// Every reading of the selected DHT, with the selected barometer's pressure.
#[embassy_executor::task]
pub async fn environmental(can: BufferedFdCanSender) -> ! {
    on_change(can, &signals::SELECTED_DHT, |reading| {
        let baro = signals::SELECTED_BARO.try_get()?;
        Some(Message::EnvironmentalData(EnvironmentalData {
            temperature: Celsius(reading.cal.temperature_c),
            humidity: reading.cal.humidity_rh,
            pressure: HPa(baro.cal.pressure_mbar),
        }))
    })
    .await
}

#[embassy_executor::task]
pub async fn status(can: BufferedFdCanSender) -> ! {
    fn convert(s: &AtomicSensorStatus) -> CanSensorStatus {
        match s.load(Ordering::Relaxed) {
            SensorStatus::Inactive => CanSensorStatus::Offline,
            SensorStatus::Active => CanSensorStatus::Online,
        }
    }

    every(can, STATUS_PERIOD, || {
        Some(Message::BoardStatus(SensorCarrierStatus {
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
        }))
    })
    .await
}

#[embassy_executor::task]
pub async fn build_info(can: BufferedFdCanSender) -> ! {
    every(can, BUILD_INFO_PERIOD, || {
        Some(Message::BuildInfo(crate::built::can_build_information()))
    })
    .await
}

/// The newest navigation state, unless the estimator has stopped.
fn fresh_state() -> Option<NavigationState> {
    signals::STATE_ESTIMATE_WATCH.try_get().filter(|state| {
        Instant::now().saturating_duration_since(Instant::from_micros(state.time_us))
            < STATE_MAX_AGE
    })
}

fn body_to_ned(state: &NavigationState) -> UnitQuaternion<f32> {
    let [w, x, y, z] = state.orientation_body_to_ned_wxyz;
    UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
}
