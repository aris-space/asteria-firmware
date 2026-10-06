//! Publishers that fill [`OUTPUTS`] from the navigation state and the sensor
//! channels; the broadcast tasks send what they publish.

use core::sync::atomic::Ordering;

use asteria_sef_core::NavigationState;
use datatypes::status::{SensorStatus as CanSensorStatus, StatusCommonMessage};
use datatypes::units::{Celsius, HPa};
use dp_sensor_carrier::{
    EnvironmentalData, ImuData, MagnetometerData, OrientationData, PositionData,
    SensorCarrierStatus, SensorsHealth, VelocityData,
};
use embassy_sync::pubsub::{DynSubscriber, WaitResult};
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use super::OUTPUTS;
use crate::calibration;
use crate::sensors::{
    AtomicSensorStatus, BARO_COUNT, BARO_STATUS, DHT_COUNT, DHT_STATUS, GNSS_STATUS, IMU_STATUS,
    MAG_COUNT, MAG_STATUS, SensorStatus,
};
use crate::signals;
use crate::tasks::state_estimation::{height_msl_referenced, launch_site};

// The fastest navigation message rate; the broadcasts throttle the slower ones.
const NAVIGATION_PERIOD: Duration = Duration::from_hz(40);
const ENVIRONMENTAL_PERIOD: Duration = Duration::from_secs(1);
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);

// The estimator publishes with every IMU sample, so an older state means it stopped.
const STATE_MAX_AGE: Duration = Duration::from_millis(100);
// The slowest sensors (DHT) read at 1 Hz; older readings are not published.
const READING_MAX_AGE: Duration = Duration::from_secs(2);

/// Orientation, IMU, velocity and position from the navigation state.
#[embassy_executor::task]
pub async fn navigation() -> ! {
    let reference = launch_site();
    let mut ticker = Ticker::every(NAVIGATION_PERIOD);
    loop {
        ticker.next().await;
        let Some(state) = fresh_state() else {
            continue;
        };
        let body_to_ned = body_to_ned(&state);

        // The CAN contract uses the NED-to-body rotation.
        let ned_to_body = body_to_ned.inverse();
        OUTPUTS.orientation.sender().send(OrientationData {
            orientation_w: ned_to_body.w,
            orientation_x: ned_to_body.i,
            orientation_y: ned_to_body.j,
            orientation_z: ned_to_body.k,
        });

        let acceleration = Vector3::from(state.specific_force_body_mps2);
        let rate_dps = Vector3::from(state.angular_rate_body_rad_s).map(f32::to_degrees);
        let acceleration_ned = body_to_ned * acceleration;
        let rate_ned_dps = body_to_ned * rate_dps;
        OUTPUTS.inertial.sender().send(ImuData {
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
        });

        let velocity_ned = Vector3::from(state.velocity_ned_mps);
        let velocity_body = ned_to_body * velocity_ned;
        OUTPUTS.velocity.sender().send(VelocityData {
            velocity_x: velocity_body.x,
            velocity_y: velocity_body.y,
            velocity_z: velocity_body.z,
            velocity_north: velocity_ned.x,
            velocity_east: velocity_ned.y,
            velocity_down: velocity_ned.z,
        });

        // Position only once GNSS has referenced the height to MSL.
        let [north_std_m, east_std_m, down_std_m] = state.position_std_ned_m;
        if height_msl_referenced(down_std_m) {
            let position = reference.unproject(state.position_ned_m);
            OUTPUTS.position.sender().send(PositionData {
                location_latitude: position.latitude_deg,
                location_longitude: position.longitude_deg,
                location_hamsl: position.height_msl_m,
                horizontal_accuracy: north_std_m.max(east_std_m),
                vertical_accuracy: down_std_m,
            });
        }
    }
}

/// The mean of the barometers' newest pressures, with every barometer reading.
#[embassy_executor::task]
pub async fn pressure() -> ! {
    let mut baro = signals::BARO_CHANNEL
        .dyn_subscriber()
        .expect("CAN: barometer subscriber slot");
    let mut pressure_hpa = [None; BARO_COUNT];
    loop {
        if let WaitResult::Message(reading) = baro.next_message().await {
            pressure_hpa[reading.raw.src.index()] =
                Some(Latest::new(reading.raw.read_ts, reading.cal.pressure_mbar));
            if let Some(mean) = mean(&pressure_hpa) {
                OUTPUTS.pressure.sender().send(HPa(mean));
            }
        }
    }
}

/// The mean of the calibrated magnetometers' newest fields, with every
/// magnetometer reading; the message promises hard- and soft-iron correction.
#[embassy_executor::task]
pub async fn magnetic_field() -> ! {
    const NT_TO_UT: f32 = 1e-3;
    let mut mag = signals::MAG_CHANNEL
        .dyn_subscriber()
        .expect("CAN: magnetometer subscriber slot");
    let mut field_ut = [None; MAG_COUNT];
    loop {
        let WaitResult::Message(reading) = mag.next_message().await else {
            continue;
        };
        if !calibration::mag::CAL
            .applied(reading.raw.src)
            .correction
            .is_calibrated()
        {
            continue;
        }
        let field = Vector3::new(reading.cal.x, reading.cal.y, reading.cal.z) * NT_TO_UT;
        field_ut[reading.raw.src.index()] = Some(Latest::new(reading.raw.read_ts, field));

        let fields = fresh(&field_ut);
        let count = fields.clone().count();
        let Some(state) = fresh_state() else {
            continue;
        };
        let body = fields.map(|field| field.value).sum::<Vector3<f32>>() / count as f32;
        let ned = body_to_ned(&state) * body;
        OUTPUTS.magnetic_field.sender().send(MagnetometerData {
            magnetic_field_x: body.x,
            magnetic_field_y: body.y,
            magnetic_field_z: body.z,
            magnetic_field_north: ned.x,
            magnetic_field_east: ned.y,
            magnetic_field_down: ned.z,
        });
    }
}

/// Temperature and humidity, with the mean pressure over the last period.
#[embassy_executor::task]
pub async fn environmental() -> ! {
    let mut baro = signals::BARO_CHANNEL
        .dyn_subscriber()
        .expect("CAN: barometer subscriber slot");
    let mut dht = signals::DHT_CHANNEL
        .dyn_subscriber()
        .expect("CAN: DHT subscriber slot");
    let mut temperature_c = [None; DHT_COUNT];
    let mut humidity_rh = [None; DHT_COUNT];
    let mut ticker = Ticker::every(ENVIRONMENTAL_PERIOD);
    loop {
        ticker.next().await;
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
        if let (Some(temperature_c), Some(humidity_rh), true) =
            (mean(&temperature_c), mean(&humidity_rh), pressure_count > 0)
        {
            OUTPUTS.environmental.sender().send(EnvironmentalData {
                temperature: Celsius(temperature_c),
                humidity: humidity_rh,
                pressure: HPa(pressure_sum_hpa / pressure_count as f32),
            });
        }
    }
}

#[embassy_executor::task]
pub async fn status() -> ! {
    fn convert(s: &AtomicSensorStatus) -> CanSensorStatus {
        match s.load(Ordering::Relaxed) {
            SensorStatus::Inactive => CanSensorStatus::Offline,
            SensorStatus::Active => CanSensorStatus::Online,
        }
    }

    let mut ticker = Ticker::every(STATUS_PERIOD);
    loop {
        OUTPUTS.status.sender().send(SensorCarrierStatus {
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
        });
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn build_info() -> ! {
    let mut ticker = Ticker::every(BUILD_INFO_PERIOD);
    loop {
        OUTPUTS
            .build_info
            .sender()
            .send(crate::built::can_build_information());
        ticker.next().await;
    }
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
