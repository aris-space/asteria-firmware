//! CAN TX/RX: subscribes to derived signals and emits hermes-can frames.
//!
//! Layout matches fw-sensor-carrier's `can_impl.rs`: a single CAN TX mutex,
//! one transmitter task per derived signal, rate-limited via a min period,
//! plus a status frame and a periodic build-info frame.

use core::sync::atomic::Ordering;

use defmt::{error, trace, warn};
use embassy_executor::Spawner;
use embassy_stm32::can::enums::BusError;
use embassy_stm32::can::frame::{self, FdFrame, Header};
use embassy_stm32::can::{Can, CanRx, CanTx};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, Ticker, TimeoutError, with_timeout};
use embedded_can::Id;
use hermes_can::messages::Message;
use hermes_can::messages::board_status::SensorStatus as CanSensorStatus;
use hermes_can::{CanDecodeError, CanEncodeError, CanMessage, next_valid_length};
use nalgebra::Vector3;

use crate::sensors::{
    AtomicSensorStatus, BAROMETER_STATUS, DHT_STATUS, GNSS_STATUS, IMU_STATUS, MAGNETOMETER_STATUS,
    SensorStatus,
};
use crate::signals;

#[allow(dead_code)]
#[derive(Debug, thiserror::Error, defmt::Format)]
pub enum CanError {
    #[error("CAN bus error")]
    Bus(BusError),

    #[error("CAN timeout")]
    Timeout(TimeoutError),

    #[error("Encoding CAN message failed")]
    Encode(#[from] CanEncodeError),

    #[error("Decoding CAN message failed")]
    Decode(#[from] CanDecodeError),

    #[error("Invalid frame or Id sent/received")]
    Other,
}

trait CanTransmitter {
    async fn transmit<M: CanMessage>(&mut self, msg: M) -> Result<(), CanError>;
}

trait CanReceiver {
    async fn recv(&mut self) -> Result<(Message, frame::Timestamp), CanError>;
}

macro_rules! impl_can_transmitter {
    ($ty:ty) => {
        impl CanTransmitter for $ty {
            async fn transmit<M: CanMessage>(&mut self, msg: M) -> Result<(), CanError> {
                let mut buf = [0u8; 64];
                let (id, len) = msg.try_write_into(&mut buf)?;
                let dlc = next_valid_length(len).ok_or(CanError::Other)?;
                let payload = &buf[..dlc];
                let frame = FdFrame::new(Header::new(id.into(), dlc as u8, false), payload)
                    .map_err(|_| CanError::Other)?;
                if let Some(pushed) = self.write_fd(&frame).await {
                    warn!("CAN dropped frame: {:?}", pushed);
                }
                Ok(())
            }
        }
    };
}

macro_rules! impl_can_receiver {
    ($ty:ty) => {
        impl CanReceiver for $ty {
            async fn recv(&mut self) -> Result<(Message, frame::Timestamp), CanError> {
                let envelope = self.read_fd().await.map_err(CanError::Bus)?;
                let frame = envelope.frame;
                let id = match frame.id() {
                    Id::Standard(id) => id,
                    Id::Extended(_) => return Err(CanError::Other),
                };
                let msg = Message::try_from_parts(*id, frame.data())?;
                Ok((msg, envelope.ts))
            }
        }
    };
}

impl_can_receiver!(Can<'_>);
impl_can_transmitter!(Can<'_>);
impl_can_receiver!(CanRx<'_>);
impl_can_transmitter!(CanTx<'_>);

const THIS_BOARD_ID: hermes_can::messages::BoardId = hermes_can::messages::BoardId::SensorCarrier;

#[embassy_executor::task]
pub async fn rx_task(mut can_rx: CanRx<'static>) -> ! {
    loop {
        match can_rx.recv().await {
            Ok((msg, _ts)) => match msg {
                Message::ResetAll(_) => {
                    warn!("CAN: ResetAll received, resetting");
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Message::ResetSpecific(x) if x.board_id == THIS_BOARD_ID => {
                    warn!("CAN: ResetSpecific received, resetting");
                    cortex_m::peripheral::SCB::sys_reset();
                }
                _ => {}
            },
            Err(err) => error!("CAN RX error: {:?}", err),
        }
    }
}

// --- TX -------------------------------------------------------------------

const TX_TIMEOUT: Duration = Duration::from_millis(100);

const fn min_period(target_hz: f32) -> Duration {
    const ALPHA: f32 = 0.2;
    Duration::from_millis((1000.0 / (target_hz * (1.0 + ALPHA))) as u64)
}

const PRESSURE_MIN_PERIOD: Duration = min_period(40.0);
const ENVIRONMENTAL_MIN_PERIOD: Duration = min_period(1.0);
const ORIENTATION_MIN_PERIOD: Duration = min_period(40.0);
const MAGNETIC_FIELD_MIN_PERIOD: Duration = min_period(10.0);
const POSITION_MIN_PERIOD: Duration = min_period(20.0);
const VELOCITY_MIN_PERIOD: Duration = min_period(20.0);
const INERTIAL_MIN_PERIOD: Duration = min_period(40.0);
const STATUS_PERIOD: Duration = Duration::from_secs(1);
const BUILD_INFO_PERIOD: Duration = Duration::from_secs(5);
const NEW_DATA_TIMEOUT: Duration = Duration::from_secs(10);

static CAN_TX: OnceLock<Mutex<ThreadModeRawMutex, CanTx<'static>>> = OnceLock::new();

pub fn spawn_tx_tasks(can_tx: CanTx<'static>, spawner: Spawner) {
    CAN_TX
        .init(Mutex::new(can_tx))
        .ok()
        .expect("CAN TX init twice");
    let can_tx = CAN_TX.try_get().expect("CAN TX not yet initialized");

    spawner.spawn(pressure_task(can_tx).expect("spawn can pressure"));
    spawner.spawn(environmental_task(can_tx).expect("spawn can env"));
    spawner.spawn(orientation_task(can_tx).expect("spawn can orientation"));
    spawner.spawn(magnetic_field_task(can_tx).expect("spawn can mag"));
    spawner.spawn(position_task(can_tx).expect("spawn can position"));
    spawner.spawn(velocity_task(can_tx).expect("spawn can velocity"));
    spawner.spawn(inertial_task(can_tx).expect("spawn can inertial"));
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
async fn pressure_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::PressureData;
    watch_loop!(
        signals::PRESSURE_FUSED_WATCH,
        PRESSURE_MIN_PERIOD,
        |pressure| {
            send(can_tx, PressureData { pressure }).await;
        }
    );
}

#[embassy_executor::task]
async fn environmental_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    watch_loop!(
        signals::ENVIRONMENTAL_WATCH,
        ENVIRONMENTAL_MIN_PERIOD,
        |env| {
            send(can_tx, env).await;
        }
    );
}

#[embassy_executor::task]
async fn orientation_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::OrientationData;
    watch_loop!(signals::ORIENTATION_WATCH, ORIENTATION_MIN_PERIOD, |q| {
        let msg = OrientationData {
            orientation_w: q.w,
            orientation_x: q.i,
            orientation_y: q.j,
            orientation_z: q.k,
        };
        send(can_tx, msg).await;
    });
}

#[embassy_executor::task]
async fn magnetic_field_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::sensor_data::MagnetometerData;
    let mut orientation_rx = signals::ORIENTATION_WATCH.anon_receiver();
    watch_loop!(
        signals::MAG_FIELD_WATCH,
        MAGNETIC_FIELD_MIN_PERIOD,
        |field| {
            let orientation = orientation_rx.try_get().unwrap_or_default();
            // nT -> uT. xyz is body-frame; orientation is body -> NED.
            let xyz = Vector3::new(field.x * 1e-3, field.y * 1e-3, field.z * 1e-3);
            let ned = orientation * xyz;
            let msg = MagnetometerData {
                magnetic_field_x: xyz.x,
                magnetic_field_y: xyz.y,
                magnetic_field_z: xyz.z,
                magnetic_field_north: ned.x,
                magnetic_field_east: ned.y,
                magnetic_field_down: ned.z,
            };
            send(can_tx, msg).await;
        }
    );
}

#[embassy_executor::task]
async fn position_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    watch_loop!(signals::POSITION_WATCH, POSITION_MIN_PERIOD, |pos| {
        send(can_tx, pos).await;
    });
}

#[embassy_executor::task]
async fn velocity_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    watch_loop!(signals::VELOCITY_WATCH, VELOCITY_MIN_PERIOD, |vel| {
        send(can_tx, vel).await;
    });
}

#[embassy_executor::task]
async fn inertial_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    watch_loop!(signals::INERTIAL_WATCH, INERTIAL_MIN_PERIOD, |imu| {
        send(can_tx, imu).await;
    });
}

#[embassy_executor::task]
async fn status_task(can_tx: &'static Mutex<ThreadModeRawMutex, CanTx<'static>>) {
    use hermes_can::messages::board_status::{
        SensorCarrierStatus, SensorsHealth, StatusCommonMessage,
    };
    let mut ticker = Ticker::every(STATUS_PERIOD);

    fn convert(s: &AtomicSensorStatus) -> CanSensorStatus {
        match s.load(Ordering::Relaxed) {
            SensorStatus::Inactive => CanSensorStatus::Offline,
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
    use hermes_can::messages::debug_info::SensorCarrierBuildInfo;
    let info = crate::build_info::BUILD_INFO.get();
    let msg = SensorCarrierBuildInfo { data: info.clone() };
    let mut ticker = Ticker::every(BUILD_INFO_PERIOD);
    loop {
        send(can_tx, msg.clone()).await;
        ticker.next().await;
    }
}
