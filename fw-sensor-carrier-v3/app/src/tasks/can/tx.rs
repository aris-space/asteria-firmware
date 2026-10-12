use core::sync::atomic::Ordering;

use asteria_sef_core::NavigationState;
use asteria_sef_light::STANDARD_GRAVITY_MPS2;
use datatypes::{
    status::{SensorStatus as CanSensorStatus, StatusCommonMessage},
    units::{Celsius, HPa},
};
use defmt::{error, trace};
use dp_sensor_carrier::{
    EnvironmentalData, ImuData, MagnetometerData, Message, OrientationData, PositionData,
    SensorCarrierStatus, SensorsHealth, VelocityData,
};
use embassy_futures::select::{Either, select};
use embassy_stm32::can::BufferedFdCanSender;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Instant, Ticker};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::{
    filters::{Filter as _, MovingAverage},
    sensors::{
        AtomicSensorStatus, BARO_STATUS, DHT_STATUS, GNSS_STATUS, IMU_STATUS, MAG_STATUS,
        SensorStatus,
    },
    signals,
    tasks::{
        can::utils::{BufferedCanTransmit as _, TryTxError},
        readout,
        state_estimation::position_reference,
    },
    types::ImuSample,
};

const POSITION_PERIOD: Duration = Duration::from_hz(20);
const VELOCITY_PERIOD: Duration = Duration::from_hz(20);
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);

const IMU_TX_HZ: u32 = 40;
const IMU_MEAN_SAMPLES: usize = (readout::imu::SAMPLE_HZ / IMU_TX_HZ as f32) as usize;
const PRESSURE_MEAN_SAMPLES: usize = (readout::baro::SAMPLE_HZ / readout::dht::SAMPLE_HZ) as usize;

const _: () = assert!(IMU_MEAN_SAMPLES > 0 && PRESSURE_MEAN_SAMPLES > 0);

#[embassy_executor::task]
pub async fn orientation(can: BufferedFdCanSender) -> ! {
    let watch = &signals::STATE_ESTIMATE;
    on_change(can, watch, |state| {
        let ned_to_body = body_to_ned(&state).inverse();
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
pub async fn imu(mut can: BufferedFdCanSender) -> ! {
    let mut readings = signals::IMU_CHANNEL
        .subscriber()
        .expect("CAN: IMU subscriber slot");
    let mut acceleration_filter = MovingAverage::<Vector3<f32>, IMU_MEAN_SAMPLES>::new();
    let mut rate_filter = MovingAverage::<Vector3<f32>, IMU_MEAN_SAMPLES>::new();
    let mut samples = 0;
    loop {
        let reading = readings.next_message_pure().await;
        if signals::SELECTED_IMU.try_get() != Some(reading.cal.src) {
            continue;
        }
        let ImuSample { accel, gyro, .. } = reading.cal;
        let acceleration_g = acceleration_filter.update(Vector3::new(accel.x, accel.y, accel.z));
        let rate_dps = rate_filter.update(Vector3::new(gyro.x, gyro.y, gyro.z));
        samples = (samples + 1) % IMU_MEAN_SAMPLES;
        if samples != 0 {
            continue;
        }
        let Some(state) = signals::STATE_ESTIMATE.try_get() else {
            continue;
        };
        let body_to_ned = body_to_ned(&state);
        let acceleration = acceleration_g * STANDARD_GRAVITY_MPS2;
        let acceleration_ned =
            body_to_ned * acceleration + Vector3::new(0.0, 0.0, STANDARD_GRAVITY_MPS2);
        let rate_ned_dps = body_to_ned * rate_dps;
        send(
            &mut can,
            Message::ImuData(ImuData {
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
            }),
        );
    }
}

#[embassy_executor::task]
pub async fn position(can: BufferedFdCanSender) -> ! {
    let reference = position_reference();
    let mut states = signals::STATE_ESTIMATE
        .receiver()
        .expect("CAN: state watch receiver slot");
    every(can, POSITION_PERIOD, || {
        let state = states.try_changed()?;
        let [north_std_m, east_std_m, down_std_m] = state.position_std_ned_m;
        let position = reference.unproject(state.position_ned_m);
        Some(Message::PositionData(PositionData {
            location_latitude: position.latitude_deg,
            location_longitude: position.longitude_deg,
            location_hamsl: position.height_msl_m,
            horizontal_accuracy: libm::hypotf(north_std_m, east_std_m),
            vertical_accuracy: down_std_m,
        }))
    })
    .await
}

#[embassy_executor::task]
pub async fn velocity(can: BufferedFdCanSender) -> ! {
    let mut states = signals::STATE_ESTIMATE
        .receiver()
        .expect("CAN: state watch receiver slot");
    every(can, VELOCITY_PERIOD, || {
        let state = states.try_changed()?;
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

#[embassy_executor::task]
pub async fn pressure(can: BufferedFdCanSender) -> ! {
    let watch = &signals::SELECTED_BARO;
    on_change(can, watch, |reading| {
        Some(Message::PressureData(HPa(reading.cal.pressure_mbar)))
    })
    .await
}

#[embassy_executor::task]
pub async fn magnetometer(can: BufferedFdCanSender) -> ! {
    const NT_TO_UT: f32 = 1e-3;
    let watch = &signals::SELECTED_MAG;
    on_change(can, watch, |reading| {
        let body = Vector3::new(reading.cal.x, reading.cal.y, reading.cal.z) * NT_TO_UT;
        let ned = body_to_ned(&signals::STATE_ESTIMATE.try_get()?) * body;
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

#[embassy_executor::task]
pub async fn environmental(mut can: BufferedFdCanSender) -> ! {
    let mut dht = signals::SELECTED_DHT
        .receiver()
        .expect("CAN: DHT watch receiver slot");
    let mut baro = signals::SELECTED_BARO
        .receiver()
        .expect("CAN: barometer watch receiver slot");
    let mut pressure_filter = MovingAverage::<f32, PRESSURE_MEAN_SAMPLES>::new();
    let mut pressure_mbar = None;
    loop {
        match select(dht.changed(), baro.changed()).await {
            Either::First(reading) => {
                let Some(pressure_mbar) = pressure_mbar else {
                    continue;
                };
                send(
                    &mut can,
                    Message::EnvironmentalData(EnvironmentalData {
                        temperature: Celsius(reading.cal.temperature_c),
                        humidity: reading.cal.humidity_rh,
                        pressure: HPa(pressure_mbar),
                    }),
                );
            }
            Either::Second(reading) => {
                pressure_mbar = Some(pressure_filter.update(reading.cal.pressure_mbar));
            }
        }
    }
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

async fn on_change<T: Clone, const N: usize>(
    mut can: BufferedFdCanSender,
    watch: &'static Watch<CriticalSectionRawMutex, T, N>,
    mut build: impl FnMut(T) -> Option<Message>,
) -> ! {
    let mut receiver = watch.receiver().expect("CAN: watch receiver slot");
    loop {
        if let Some(msg) = build(receiver.changed().await) {
            send(&mut can, msg);
        }
    }
}

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

fn send(can: &mut BufferedFdCanSender, msg: Message) {
    match can.try_transmit(msg) {
        Ok(()) => {}
        Err(TryTxError::Full) => trace!("CAN: TX buffer full, frame dropped"),
        Err(err) => error!("CAN: TX error: {:?}", err),
    }
}

fn body_to_ned(state: &NavigationState) -> UnitQuaternion<f32> {
    let [w, x, y, z] = state.orientation_body_to_ned_wxyz;
    UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z))
}
