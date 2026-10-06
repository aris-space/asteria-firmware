//! The only CAN transmitter. One task owns `CanTx` and sends every
//! `dp-sensor-carrier` message at its specified rate from one 40 Hz tick.

use core::sync::atomic::Ordering;

use asteria_state_estimation::{GeodeticReference, NavigationState};
use can_utils::rxtx::TypedCanTransmit as _;
use datatypes::status::{SensorStatus as CanSensorStatus, StatusCommonMessage};
use datatypes::units::{Celsius, HPa};
use defmt::{error, info, warn};
use dp_sensor_carrier::{
    EnvironmentalData, ImuData, MagnetometerData, Message, OrientationData, PositionData,
    SensorCarrierStatus, SensorsHealth, VelocityData,
};
use embassy_stm32::can::CanTx;
use embassy_sync::pubsub::{DynSubscriber, WaitResult};
use embassy_time::{Duration, Instant, Ticker, with_timeout};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::calibration;
use crate::sensors::{
    AtomicSensorStatus, BARO_COUNT, BARO_STATUS, DHT_COUNT, DHT_STATUS, GNSS_STATUS, IMU_STATUS,
    MAG_COUNT, MAG_STATUS, SensorStatus,
};
use crate::signals;
use crate::tasks::state_estimation::{height_msl_referenced, launch_site};
use crate::types::{BaroReading, DhtReading, MagReading};

// The fastest message rate; every other period is a multiple of it.
const TICK: Duration = Duration::from_hz(40);
const ORIENTATION_PERIOD: Duration = Duration::from_hz(40);
const IMU_PERIOD: Duration = Duration::from_hz(40);
const PRESSURE_PERIOD: Duration = Duration::from_hz(40);
const POSITION_PERIOD: Duration = Duration::from_hz(20);
const VELOCITY_PERIOD: Duration = Duration::from_hz(20);
const MAGNETOMETER_PERIOD: Duration = Duration::from_hz(10);
const ENVIRONMENTAL_PERIOD: Duration = Duration::from_secs(1);
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);

const TX_TIMEOUT: Duration = Duration::from_millis(100);
// The estimator publishes with every sample, so an older state means it stopped.
const STATE_MAX_AGE: Duration = Duration::from_millis(100);
// The slowest sensors (DHT) read at 1 Hz; older readings are not sent.
const READING_MAX_AGE: Duration = Duration::from_secs(2);
const NEW_DATA_TIMEOUT: Duration = Duration::from_secs(10);

#[embassy_executor::task]
pub async fn task(can_tx: CanTx<'static>) -> ! {
    let mut estimates = signals::STATE_ESTIMATE_WATCH
        .receiver()
        .expect("CAN: state estimate receiver available");
    let mut sensors = Sensors::new();
    let reference = launch_site();
    let mut bus = Bus {
        can_tx,
        healthy: true,
    };
    let mut state: Option<(Instant, NavigationState)> = None;
    let mut estimate_missing = false;
    let mut ticker = Ticker::every(TICK);
    let mut tick: u64 = 0;
    loop {
        ticker.next().await;
        let now = Instant::now();
        let this_tick = tick;
        tick += 1;
        let due = |period: Duration| this_tick % (period.as_ticks() / TICK.as_ticks()) == 0;

        if let Some(new) = estimates.try_changed() {
            state = Some((now, new));
            estimate_missing = false;
        }
        let last_estimate = state
            .as_ref()
            .map(|(at, _)| now.saturating_duration_since(*at));
        if !estimate_missing && last_estimate.is_none_or(|age| age >= NEW_DATA_TIMEOUT) {
            error!("CAN: no state estimate for 10 s");
            estimate_missing = true;
        }
        let fresh_state = state
            .as_ref()
            .filter(|(at, _)| now.saturating_duration_since(*at) < STATE_MAX_AGE)
            .map(|(_, state)| state);
        sensors.update();

        if let Some(state) = fresh_state {
            if due(ORIENTATION_PERIOD) {
                bus.send(Message::OrientationData(orientation(state))).await;
            }
            if due(IMU_PERIOD) {
                bus.send(Message::ImuData(imu(state))).await;
            }
            if due(VELOCITY_PERIOD) {
                bus.send(Message::VelocityData(velocity(state))).await;
            }
            if due(POSITION_PERIOD)
                && let Some(position) = position(&reference, state)
            {
                bus.send(Message::PositionData(position)).await;
            }
            if due(MAGNETOMETER_PERIOD)
                && let Some(field) = sensors.magnetic_field(state, now)
            {
                bus.send(Message::MagnetometerData(field)).await;
            }
        }
        if due(PRESSURE_PERIOD)
            && let Some(pressure_hpa) = mean(&sensors.pressure_hpa, now)
        {
            bus.send(Message::PressureData(HPa(pressure_hpa))).await;
        }
        if due(ENVIRONMENTAL_PERIOD)
            && let Some(environment) = sensors.environment(now)
        {
            bus.send(Message::EnvironmentalData(environment)).await;
        }
        if due(STATUS_PERIOD) {
            bus.send(Message::BoardStatus(status())).await;
        }
        if due(BUILD_INFO_PERIOD) {
            bus.send(Message::BuildInfo(crate::built::can_build_information()))
                .await;
        }

        // With a dead bus every send waits out its timeout; start the tick
        // afresh rather than catch up with a burst of stale frames.
        if !bus.healthy {
            ticker.reset();
        }
    }
}

struct Bus {
    can_tx: CanTx<'static>,
    healthy: bool,
}

impl Bus {
    /// Sends one frame, reporting only when the bus stops or starts taking frames.
    async fn send(&mut self, msg: Message) {
        match with_timeout(TX_TIMEOUT, self.can_tx.transmit(msg)).await {
            Ok(Ok(())) => {
                if !self.healthy {
                    info!("CAN: bus takes frames again");
                    self.healthy = true;
                }
            }
            Ok(Err(err)) => error!("CAN: TX error: {:?}", err),
            Err(_) => {
                if self.healthy {
                    warn!(
                        "CAN: TX timed out after {} ms, is the bus connected?",
                        TX_TIMEOUT.as_millis()
                    );
                    self.healthy = false;
                }
            }
        }
    }
}

/// A value and when its sensor read it.
#[derive(Clone, Copy)]
struct Latest<T> {
    read_ts: Instant,
    value: T,
}

/// The newest reading of every sensor the navigation state does not carry.
struct Sensors {
    baro_rx: DynSubscriber<'static, BaroReading>,
    mag_rx: DynSubscriber<'static, MagReading>,
    dht_rx: DynSubscriber<'static, DhtReading>,
    pressure_hpa: [Option<Latest<f32>>; BARO_COUNT],
    // Calibrated magnetometers only, in µT and body axes.
    field_ut: [Option<Latest<[f32; 3]>>; MAG_COUNT],
    temperature_c: [Option<Latest<f32>>; DHT_COUNT],
    humidity_rh: [Option<Latest<f32>>; DHT_COUNT],
    // Pressure since the last environmental frame, which carries its mean.
    pressure_sum_hpa: f32,
    pressure_count: u32,
}

impl Sensors {
    fn new() -> Self {
        Self {
            baro_rx: signals::BARO_CHANNEL
                .dyn_subscriber()
                .expect("CAN: subscriber slot"),
            mag_rx: signals::MAG_CHANNEL
                .dyn_subscriber()
                .expect("CAN: subscriber slot"),
            dht_rx: signals::DHT_CHANNEL
                .dyn_subscriber()
                .expect("CAN: subscriber slot"),
            pressure_hpa: [None; BARO_COUNT],
            field_ut: [None; MAG_COUNT],
            temperature_c: [None; DHT_COUNT],
            humidity_rh: [None; DHT_COUNT],
            pressure_sum_hpa: 0.0,
            pressure_count: 0,
        }
    }

    /// Takes every reading that arrived since the last tick.
    fn update(&mut self) {
        const NT_TO_UT: f32 = 1e-3;
        while let Some(message) = self.baro_rx.try_next_message() {
            if let WaitResult::Message(reading) = message {
                let pressure_hpa = reading.cal.pressure_mbar;
                self.pressure_hpa[reading.raw.src.index()] = Some(Latest {
                    read_ts: reading.raw.read_ts,
                    value: pressure_hpa,
                });
                self.pressure_sum_hpa += pressure_hpa;
                self.pressure_count += 1;
            }
        }
        while let Some(message) = self.mag_rx.try_next_message() {
            if let WaitResult::Message(reading) = message
                && calibration::mag::CAL
                    .applied(reading.raw.src)
                    .correction
                    .is_calibrated()
            {
                let field = [reading.cal.x, reading.cal.y, reading.cal.z];
                self.field_ut[reading.raw.src.index()] = Some(Latest {
                    read_ts: reading.raw.read_ts,
                    value: field.map(|nt| nt * NT_TO_UT),
                });
            }
        }
        while let Some(message) = self.dht_rx.try_next_message() {
            if let WaitResult::Message(reading) = message {
                let index = reading.raw.src.index();
                let read_ts = reading.raw.read_ts;
                self.temperature_c[index] = Some(Latest {
                    read_ts,
                    value: reading.cal.temperature_c,
                });
                self.humidity_rh[index] = Some(Latest {
                    read_ts,
                    value: reading.cal.humidity_rh,
                });
            }
        }
    }

    /// The mean calibrated field, in body and NED axes.
    fn magnetic_field(&self, state: &NavigationState, now: Instant) -> Option<MagnetometerData> {
        let mut sum = [0.0; 3];
        let mut count = 0;
        for latest in fresh(&self.field_ut, now) {
            for (sum, value) in sum.iter_mut().zip(latest.value) {
                *sum += value;
            }
            count += 1;
        }
        if count == 0 {
            return None;
        }
        let body = Vector3::from(sum) / count as f32;
        let ned = body_to_ned(state) * body;
        Some(MagnetometerData {
            magnetic_field_x: body.x,
            magnetic_field_y: body.y,
            magnetic_field_z: body.z,
            magnetic_field_north: ned.x,
            magnetic_field_east: ned.y,
            magnetic_field_down: ned.z,
        })
    }

    /// Temperature and humidity, with the mean pressure since the last
    /// environmental frame.
    fn environment(&mut self, now: Instant) -> Option<EnvironmentalData> {
        let pressure_hpa =
            (self.pressure_count > 0).then(|| self.pressure_sum_hpa / self.pressure_count as f32);
        self.pressure_sum_hpa = 0.0;
        self.pressure_count = 0;
        Some(EnvironmentalData {
            temperature: Celsius(mean(&self.temperature_c, now)?),
            humidity: mean(&self.humidity_rh, now)?,
            pressure: HPa(pressure_hpa?),
        })
    }
}

/// The values no older than [`READING_MAX_AGE`].
fn fresh<T>(values: &[Option<Latest<T>>], now: Instant) -> impl Iterator<Item = &Latest<T>> {
    values
        .iter()
        .flatten()
        .filter(move |latest| now.saturating_duration_since(latest.read_ts) < READING_MAX_AGE)
}

/// The mean of the values no older than [`READING_MAX_AGE`].
fn mean(values: &[Option<Latest<f32>>], now: Instant) -> Option<f32> {
    let (sum, count) = fresh(values, now).fold((0.0, 0), |(sum, count), latest| {
        (sum + latest.value, count + 1)
    });
    (count > 0).then(|| sum / count as f32)
}

fn body_to_ned(state: &NavigationState) -> UnitQuaternion<f32> {
    let [w, x, y, z] = state.orientation_body_to_ned_wxyz;
    UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
}

fn orientation(state: &NavigationState) -> OrientationData {
    // The CAN contract uses the NED-to-body rotation.
    let ned_to_body = body_to_ned(state).inverse();
    OrientationData {
        orientation_w: ned_to_body.w,
        orientation_x: ned_to_body.i,
        orientation_y: ned_to_body.j,
        orientation_z: ned_to_body.k,
    }
}

fn velocity(state: &NavigationState) -> VelocityData {
    let ned = Vector3::from(state.velocity_ned_mps);
    let body = body_to_ned(state).inverse() * ned;
    VelocityData {
        velocity_x: body.x,
        velocity_y: body.y,
        velocity_z: body.z,
        velocity_north: ned.x,
        velocity_east: ned.y,
        velocity_down: ned.z,
    }
}

fn imu(state: &NavigationState) -> ImuData {
    let body_to_ned = body_to_ned(state);
    let acceleration = Vector3::from(state.specific_force_body_mps2);
    let rate_dps = Vector3::from(state.angular_rate_body_rad_s).map(f32::to_degrees);
    let acceleration_ned = body_to_ned * acceleration;
    let rate_ned_dps = body_to_ned * rate_dps;
    ImuData {
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
    }
}

/// The position, once GNSS has referenced the height to MSL.
fn position(reference: &GeodeticReference, state: &NavigationState) -> Option<PositionData> {
    let [north_std_m, east_std_m, down_std_m] = state.position_std_ned_m;
    if !height_msl_referenced(down_std_m) {
        return None;
    }
    let position = reference.unproject(state.position_ned_m);
    Some(PositionData {
        location_latitude: position.latitude_deg,
        location_longitude: position.longitude_deg,
        location_hamsl: position.height_msl_m,
        horizontal_accuracy: north_std_m.max(east_std_m),
        vertical_accuracy: down_std_m,
    })
}

fn status() -> SensorCarrierStatus {
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
