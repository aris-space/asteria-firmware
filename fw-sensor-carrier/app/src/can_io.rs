use crate::drivers::magnetic_field::MAGNETIC_FIELD_WATCH;
use crate::sensors::{
    AtomicSensorStatus, BAROMETER_BUS_1_STATUS, BAROMETER_BUS_2_STATUS, DHT_BUS_1_STATUS,
    DHT_BUS_2_STATUS, GPS1_STATUS, GPS2_STATUS, IMU1_STATUS, IMU2_STATUS,
    MAGNETOMETER_BUS_1_STATUS, MAGNETOMETER_BUS_2_STATUS,
};
use can_utils::broadcast::Broadcast;
use can_utils::rxtx::TypedCanReceive;
use core::sync::atomic::Ordering;
use data_core::can::{hal::CanDecode, sparse_decodable_can_message};
use datatypes::status::{BoardId, BuildInformationCommon, SensorStatus, StatusCommonMessage};
use datatypes::units::HPa;
use dp_sensor_carrier::{
    EnvironmentalData, ImuData, MagnetometerData, Message, OrientationData, PositionData,
    SensorCarrierStatus, SensorsHealth, VelocityData,
};
use embassy_stm32::can::CanRx;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};
use embedded_utils::fmt::*;
use nalgebra::{UnitQuaternion, Vector3};

const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);
const IDLE_WARN_THRESHOLD: Duration = Duration::from_secs(10);
const STALE_MONITOR_TICK: Duration = Duration::from_secs(1);

const THIS_BOARD_ID: BoardId = BoardId::SensorCarrier;

// Messages received on the bus that the Sensor Carrier cares about.
sparse_decodable_can_message! {
    enum ReceivedMessage {
        ResetAll(dp_system_management::Message::ResetAll),
        ResetSpecific(dp_system_management::Message::ResetSpecific),
    }
}

const _: () = {
    const FILTER_COUNT: usize = 28;
    if ReceivedMessage::SUPPORTED_IDS.len() >= FILTER_COUNT - 1 {
        core::panic!("Too many receiving can ids");
    }
};

// max_freq_hz = target_rate * 1.2 of threshold
// min_freq_hz = 0.0 -> 1000/0 = inf, so resend timeout saturates to u64::MAX ms
#[derive(Broadcast)]
#[broadcast(loop_type = "can_utils::broadcast::ResponsiveLoop")]
pub struct Outputs {
    #[broadcast(
        map = "Message::PressureData(HPa(#value))",
        min_freq_hz = 0.00,
        max_freq_hz = 48.0
    )]
    pub pressure: Watch<ThreadModeRawMutex, f32, 3>,

    #[broadcast(
        map = "Message::EnvironmentalData(#value)",
        min_freq_hz = 0.0,
        max_freq_hz = 1.2
    )]
    pub environmental: Watch<ThreadModeRawMutex, EnvironmentalData, 3>,

    #[broadcast(
        map = "Message::OrientationData(OrientationData { orientation_w: #value.w, orientation_x: #value.i, orientation_y: #value.j, orientation_z: #value.k })",
        min_freq_hz = 0.0,
        max_freq_hz = 48.0
    )]
    pub orientation: Watch<ThreadModeRawMutex, UnitQuaternion<f32>, 3>,

    #[broadcast(
        map = "Message::MagnetometerData(#value)",
        min_freq_hz = 0.0,
        max_freq_hz = 12.0
    )]
    pub magnetic_field: Watch<ThreadModeRawMutex, MagnetometerData, 2>,

    #[broadcast(
        map = "Message::PositionData(#value)",
        min_freq_hz = 0.0,
        max_freq_hz = 24.0
    )]
    pub position: Watch<ThreadModeRawMutex, PositionData, 3>,

    #[broadcast(
        map = "Message::VelocityData(#value)",
        min_freq_hz = 0.0,
        max_freq_hz = 24.0
    )]
    pub velocity: Watch<ThreadModeRawMutex, VelocityData, 3>,

    #[broadcast(
        map = "Message::ImuData(#value)",
        min_freq_hz = 0.0,
        max_freq_hz = 48.0
    )]
    pub inertial: Watch<ThreadModeRawMutex, ImuData, 3>,

    #[broadcast(
        map = "Message::BoardStatus(#value)",
        min_freq_hz = 0.0,
        max_freq_hz = 1.2
    )]
    pub status: Watch<ThreadModeRawMutex, SensorCarrierStatus, 2>,

    #[broadcast(
        map = "Message::BuildInfo(#value)",
        min_freq_hz = 0.0,
        max_freq_hz = 0.24
    )]
    pub build_info: Watch<ThreadModeRawMutex, BuildInformationCommon, 1>,
}

pub static OUTPUTS: Outputs = Outputs {
    pressure: Watch::new(),
    environmental: Watch::new(),
    orientation: Watch::new(),
    magnetic_field: Watch::new(),
    position: Watch::new(),
    velocity: Watch::new(),
    inertial: Watch::new(),
    status: Watch::new(),
    build_info: Watch::new(),
};

#[embassy_executor::task]
pub async fn can_rx_task(mut can_rx: CanRx<'static>) -> ! {
    loop {
        match can_rx.recv::<ReceivedMessage>().await {
            Ok(msg) => match msg {
                ReceivedMessage::ResetAll(_) => {
                    warn!("Received ResetAll message, resetting Sensor Carrier");
                    reset_now();
                }
                ReceivedMessage::ResetSpecific(board_id) => {
                    if board_id == THIS_BOARD_ID {
                        warn!("Received ResetSpecific message, resetting Sensor Carrier");
                        reset_now();
                    }
                }
            },
            Err(err) => {
                error!("CAN RX error: {:?}", err);
            }
        }
    }
}

#[embassy_executor::task]
pub async fn magnetic_field_publisher() {
    let mut mag_rx = MAGNETIC_FIELD_WATCH
        .receiver()
        .expect("failed to get magnetic field watch");
    let mut orientation_anon = OUTPUTS.orientation.anon_receiver();
    let sender = OUTPUTS.magnetic_field.sender();

    loop {
        let (_ts, field) = mag_rx.changed().await;
        let orientation = orientation_anon.try_get().unwrap_or_default();

        // convert nT → µT
        let xyz = Vector3::new(
            field.x_nt() as f32 * 1e-3,
            field.y_nt() as f32 * 1e-3,
            field.z_nt() as f32 * 1e-3,
        );

        // passive rotation NED → body
        let ned = orientation.inverse() * xyz;

        sender.send(MagnetometerData {
            magnetic_field_x: xyz.x,
            magnetic_field_y: xyz.y,
            magnetic_field_z: xyz.z,
            magnetic_field_north: ned.x,
            magnetic_field_east: ned.y,
            magnetic_field_down: ned.z,
        });
    }
}

#[embassy_executor::task]
pub async fn status_publisher() {
    let sender = OUTPUTS.status.sender();
    let mut ticker = Ticker::every(STATUS_PERIOD);

    let convert = |s: &AtomicSensorStatus| match s.load(Ordering::Relaxed) {
        crate::sensors::SensorStatus::Inactive => SensorStatus::Offline,
        crate::sensors::SensorStatus::Active => SensorStatus::Online,
    };

    loop {
        let micros_since_restart = embassy_time::Instant::now().as_micros();
        let snapshot = SensorCarrierStatus {
            common: StatusCommonMessage {
                errors: 0, // TODO sometime else
                micros_since_restart,
            },
            sensor_health: SensorsHealth {
                barometer_bus_1: convert(&BAROMETER_BUS_1_STATUS),
                barometer_bus_2: convert(&BAROMETER_BUS_2_STATUS),
                dht_bus_1: convert(&DHT_BUS_1_STATUS),
                dht_bus_2: convert(&DHT_BUS_2_STATUS),
                magnetometer_bus_1: convert(&MAGNETOMETER_BUS_1_STATUS),
                magnetometer_bus_2: convert(&MAGNETOMETER_BUS_2_STATUS),
                imu_1: convert(&IMU1_STATUS),
                imu_2: convert(&IMU2_STATUS),
                gps_1: convert(&GPS1_STATUS),
                gps_2: convert(&GPS2_STATUS),
            },
        };
        sender.send(snapshot);
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn build_info_publisher() {
    let sender = OUTPUTS.build_info.sender();
    let mut ticker = Ticker::every(BUILD_INFO_PERIOD);
    loop {
        sender.send(crate::build_info::BUILD_INFO.get().clone());
        ticker.next().await;
    }
}

#[embassy_executor::task]
pub async fn stale_data_monitor() {
    let mut pressure_rx = OUTPUTS.pressure.receiver().expect("pressure rx");
    let mut environmental_rx = OUTPUTS.environmental.receiver().expect("environmental rx");
    let mut orientation_rx = OUTPUTS.orientation.receiver().expect("orientation rx");
    let mut magnetic_rx = OUTPUTS.magnetic_field.receiver().expect("magnetic rx");
    let mut position_rx = OUTPUTS.position.receiver().expect("position rx");
    let mut velocity_rx = OUTPUTS.velocity.receiver().expect("velocity rx");
    let mut inertial_rx = OUTPUTS.inertial.receiver().expect("inertial rx");

    let now = embassy_time::Instant::now();
    let mut last_pressure = now;
    let mut last_environmental = now;
    let mut last_orientation = now;
    let mut last_magnetic = now;
    let mut last_position = now;
    let mut last_velocity = now;
    let mut last_inertial = now;

    let mut ticker = Ticker::every(STALE_MONITOR_TICK);
    loop {
        ticker.next().await;
        let now = embassy_time::Instant::now();

        if pressure_rx.try_changed().is_some() {
            last_pressure = now;
        }
        if environmental_rx.try_changed().is_some() {
            last_environmental = now;
        }
        if orientation_rx.try_changed().is_some() {
            last_orientation = now;
        }
        if magnetic_rx.try_changed().is_some() {
            last_magnetic = now;
        }
        if position_rx.try_changed().is_some() {
            last_position = now;
        }
        if velocity_rx.try_changed().is_some() {
            last_velocity = now;
        }
        if inertial_rx.try_changed().is_some() {
            last_inertial = now;
        }

        // After warning, bump the timestamp so the same stream only warns once per IDLE_WARN_THRESHOLD.
        if now.saturating_duration_since(last_pressure) >= IDLE_WARN_THRESHOLD {
            error!("No pressure data for >{} s", IDLE_WARN_THRESHOLD.as_secs());
            last_pressure = now;
        }
        if now.saturating_duration_since(last_environmental) >= IDLE_WARN_THRESHOLD {
            error!(
                "No environmental data for >{} s",
                IDLE_WARN_THRESHOLD.as_secs()
            );
            last_environmental = now;
        }
        if now.saturating_duration_since(last_orientation) >= IDLE_WARN_THRESHOLD {
            error!(
                "No orientation data for >{} s",
                IDLE_WARN_THRESHOLD.as_secs()
            );
            last_orientation = now;
        }
        if now.saturating_duration_since(last_magnetic) >= IDLE_WARN_THRESHOLD {
            error!(
                "No magnetic field data for >{} s",
                IDLE_WARN_THRESHOLD.as_secs()
            );
            last_magnetic = now;
        }
        if now.saturating_duration_since(last_position) >= IDLE_WARN_THRESHOLD {
            error!("No position data for >{} s", IDLE_WARN_THRESHOLD.as_secs());
            last_position = now;
        }
        if now.saturating_duration_since(last_velocity) >= IDLE_WARN_THRESHOLD {
            error!("No velocity data for >{} s", IDLE_WARN_THRESHOLD.as_secs());
            last_velocity = now;
        }
        if now.saturating_duration_since(last_inertial) >= IDLE_WARN_THRESHOLD {
            error!("No inertial data for >{} s", IDLE_WARN_THRESHOLD.as_secs());
            last_inertial = now;
        }
    }
}

fn reset_now() -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}
