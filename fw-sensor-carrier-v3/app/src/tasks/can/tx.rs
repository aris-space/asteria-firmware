//! CAN transmission of every `dp-sensor-carrier` message.
//!
//! Each message has its own task that builds it at the message's rate and
//! hands it to the CAN peripheral's TX buffer without waiting; the driver's
//! interrupt moves frames from there into the hardware. With a dead bus the
//! buffer fills and new frames are dropped.

use core::sync::atomic::{AtomicBool, Ordering};

use asteria_sef_core::NavigationState;
use data_core::can::hal::CanEncode as _;
use datatypes::status::{SensorStatus as CanSensorStatus, StatusCommonMessage};
use datatypes::units::{Celsius, HPa};
use defmt::{error, info, warn};
use dp_sensor_carrier::{
    EnvironmentalData, ImuData, MagnetometerData, Message, OrientationData, PositionData,
    SensorCarrierStatus, SensorsHealth, VelocityData,
};
use embassy_stm32::can::BufferedFdCanSender;
use embassy_stm32::can::frame::{FdFrame, Header};
use embassy_sync::pubsub::{DynSubscriber, WaitResult};
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::calibration;
use crate::sensors::{
    AtomicSensorStatus, BARO_COUNT, BARO_STATUS, DHT_COUNT, DHT_STATUS, GNSS_STATUS, IMU_STATUS,
    MAG_COUNT, MAG_STATUS, SensorStatus,
};
use crate::signals;
use crate::tasks::state_estimation::{height_msl_referenced, launch_site};

const ORIENTATION_PERIOD: Duration = Duration::from_hz(40);
const IMU_PERIOD: Duration = Duration::from_hz(40);
const PRESSURE_PERIOD: Duration = Duration::from_hz(40);
const POSITION_PERIOD: Duration = Duration::from_hz(20);
const VELOCITY_PERIOD: Duration = Duration::from_hz(20);
const MAGNETOMETER_PERIOD: Duration = Duration::from_hz(10);
const ENVIRONMENTAL_PERIOD: Duration = Duration::from_secs(1);
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);

// The estimator publishes with every IMU sample, so an older state means it stopped.
const STATE_MAX_AGE: Duration = Duration::from_millis(100);
// The slowest sensors (DHT) read at 1 Hz; older readings are not sent.
const READING_MAX_AGE: Duration = Duration::from_secs(2);

// Whether the last frame found the TX buffer full, so the change is logged once.
static TX_BUFFER_FULL: AtomicBool = AtomicBool::new(false);

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
    let full = can.try_write(frame).is_err();
    if full != TX_BUFFER_FULL.swap(full, Ordering::Relaxed) {
        if full {
            warn!("CAN: TX buffer full, is the bus connected?");
        } else {
            info!("CAN: bus takes frames again");
        }
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

/// The mean of the barometers' newest pressures.
#[embassy_executor::task]
pub async fn pressure(can: BufferedFdCanSender) -> ! {
    let mut baro = signals::BARO_CHANNEL
        .dyn_subscriber()
        .expect("CAN: barometer subscriber slot");
    let mut pressure_hpa = [None; BARO_COUNT];
    every(can, PRESSURE_PERIOD, || {
        drain(&mut baro, |reading| {
            pressure_hpa[reading.raw.src.index()] =
                Some(Latest::new(reading.raw.read_ts, reading.cal.pressure_mbar));
        });
        Some(Message::PressureData(HPa(mean(&pressure_hpa)?)))
    })
    .await
}

/// The mean of the calibrated magnetometers' newest fields; the message
/// promises hard- and soft-iron correction.
#[embassy_executor::task]
pub async fn magnetometer(can: BufferedFdCanSender) -> ! {
    const NT_TO_UT: f32 = 1e-3;
    let mut mag = signals::MAG_CHANNEL
        .dyn_subscriber()
        .expect("CAN: magnetometer subscriber slot");
    let mut field_ut = [None; MAG_COUNT];
    every(can, MAGNETOMETER_PERIOD, || {
        drain(&mut mag, |reading| {
            let cal = calibration::mag::CAL.applied(reading.raw.src);
            if cal.correction.is_calibrated() {
                let field = Vector3::new(reading.cal.x, reading.cal.y, reading.cal.z) * NT_TO_UT;
                field_ut[reading.raw.src.index()] = Some(Latest::new(reading.raw.read_ts, field));
            }
        });
        let fields = fresh(&field_ut);
        let count = fields.clone().count();
        if count == 0 {
            return None;
        }
        let body = fields.map(|field| field.value).sum::<Vector3<f32>>() / count as f32;
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

/// Temperature and humidity, with the mean pressure over the last period.
#[embassy_executor::task]
pub async fn environmental(can: BufferedFdCanSender) -> ! {
    let mut baro = signals::BARO_CHANNEL
        .dyn_subscriber()
        .expect("CAN: barometer subscriber slot");
    let mut dht = signals::DHT_CHANNEL
        .dyn_subscriber()
        .expect("CAN: DHT subscriber slot");
    let mut temperature_c = [None; DHT_COUNT];
    let mut humidity_rh = [None; DHT_COUNT];
    every(can, ENVIRONMENTAL_PERIOD, || {
        let (mut pressure_sum_hpa, mut pressure_count) = (0.0, 0);
        drain(&mut baro, |reading| {
            pressure_sum_hpa += reading.cal.pressure_mbar;
            pressure_count += 1;
        });
        drain(&mut dht, |reading| {
            let index = reading.raw.src.index();
            temperature_c[index] =
                Some(Latest::new(reading.raw.read_ts, reading.cal.temperature_c));
            humidity_rh[index] = Some(Latest::new(reading.raw.read_ts, reading.cal.humidity_rh));
        });
        if pressure_count == 0 {
            return None;
        }
        Some(Message::EnvironmentalData(EnvironmentalData {
            temperature: Celsius(mean(&temperature_c)?),
            humidity: mean(&humidity_rh)?,
            pressure: HPa(pressure_sum_hpa / pressure_count as f32),
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

/// Takes every reading that arrived since the last call.
fn drain<T: Clone>(subscriber: &mut DynSubscriber<'static, T>, mut take: impl FnMut(T)) {
    while let Some(message) = subscriber.try_next_message() {
        if let WaitResult::Message(reading) = message {
            take(reading);
        }
    }
}

/// A value and when its sensor read it.
#[derive(Clone, Copy)]
struct Latest<T> {
    read_ts: Instant,
    value: T,
}

impl<T> Latest<T> {
    fn new(read_ts: Instant, value: T) -> Self {
        Self { read_ts, value }
    }
}

/// The values no older than [`READING_MAX_AGE`].
fn fresh<T>(values: &[Option<Latest<T>>]) -> impl Iterator<Item = &Latest<T>> + Clone {
    let now = Instant::now();
    values
        .iter()
        .flatten()
        .filter(move |latest| now.saturating_duration_since(latest.read_ts) < READING_MAX_AGE)
}

/// The mean of the values no older than [`READING_MAX_AGE`].
fn mean(values: &[Option<Latest<f32>>]) -> Option<f32> {
    let (sum, count) = fresh(values).fold((0.0, 0), |(sum, count), latest| {
        (sum + latest.value, count + 1)
    });
    (count > 0).then(|| sum / count as f32)
}
