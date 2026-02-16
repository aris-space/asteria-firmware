#![allow(dead_code)]
use crate::drivers::inertial::ORIENTATION_WATCH;
use crate::sensors::gnss::PvtData;
use crate::sensors::SensorId;
use embassy_sync::{
    blocking_mutex::raw::{NoopRawMutex, ThreadModeRawMutex},
    mutex::Mutex,
    once_lock::OnceLock,
    pubsub::{ImmediatePublisher, PubSubChannel},
    watch::{Sender, Watch},
};
use embassy_time::{Duration, Instant};
use embedded_utils::error;
use hermes_can::messages::sensor_data::{PositionData, VelocityData};
use hermes_can::messages::system_management::UTCTimeUpdate;
use nalgebra::{UnitQuaternion, Vector3};

pub const CAP: usize = 10;
pub const PUB: usize = 1;
pub const SUB: usize = 2;
pub const WATCH: usize = 3;

pub static POSITION_WATCH: Watch<ThreadModeRawMutex, PositionData, WATCH> = Watch::new();
pub static POSITION_PUBSUB: PubSubChannel<ThreadModeRawMutex, PositionData, CAP, SUB, PUB> =
    PubSubChannel::new();

pub static VELOCITY_WATCH: Watch<ThreadModeRawMutex, VelocityData, WATCH> = Watch::new();
pub static VELOCITY_PUBSUB: PubSubChannel<ThreadModeRawMutex, VelocityData, CAP, SUB, PUB> =
    PubSubChannel::new();

pub static TIME_WATCH: Watch<ThreadModeRawMutex, UTCTimeUpdate, WATCH> = Watch::new();

pub static POSITION_VELOCITY_TIME_DRIVER: OnceLock<PositionVelocityTimeDriver> = OnceLock::new();

pub struct PositionVelocityTimeDriver<'a> {
    pos_publisher: ImmediatePublisher<'a, ThreadModeRawMutex, PositionData, CAP, SUB, PUB>,
    pos_watch: Sender<'a, ThreadModeRawMutex, PositionData, WATCH>,

    vel_publisher: ImmediatePublisher<'a, ThreadModeRawMutex, VelocityData, CAP, SUB, PUB>,
    vel_watch: Sender<'a, ThreadModeRawMutex, VelocityData, WATCH>,

    shared: Mutex<NoopRawMutex, TimeoutSelector>,
}

impl<'a> PositionVelocityTimeDriver<'a> {
    pub fn new(
        pos_publisher: ImmediatePublisher<'a, ThreadModeRawMutex, PositionData, CAP, SUB, PUB>,
        pos_watch: Sender<'a, ThreadModeRawMutex, PositionData, WATCH>,
        vel_publisher: ImmediatePublisher<'a, ThreadModeRawMutex, VelocityData, CAP, SUB, PUB>,
        vel_watch: Sender<'a, ThreadModeRawMutex, VelocityData, WATCH>,
    ) -> Self {
        Self {
            pos_publisher,
            pos_watch,
            vel_publisher,
            vel_watch,
            shared: Mutex::new(TimeoutSelector::new(Duration::from_secs(2))),
        }
    }

    pub async fn update(&self, data: PvtData, ts: Instant, sensor_id: SensorId) {
        if !matches!(sensor_id, SensorId::Gps1 | SensorId::Gps2) {
            error!("Invalid sensor id for position update: {:?}", sensor_id);
            return;
        }

        if !self.shared.lock().await.accept(sensor_id, ts) {
            return;
        }

        let pos_data = PositionData {
            location_latitude: data.lat_deg,
            location_longitude: data.lon_deg,
            location_hamsl: data.height_msl,
            horizontal_accuracy: data.horiz_accuracy as f32 / 1000.0, // units are in mm, we want m
            vertical_accuracy: data.vert_accuracy as f32 / 1000.0,
        };

        let orientation = ORIENTATION_WATCH
            .anon_receiver()
            .try_get()
            .unwrap_or(UnitQuaternion::identity());

        // rotate velocity into frame
        let v_inertial = Vector3::new(data.vel_north, data.vel_east, data.vel_down);

        // we need to rotate from inertial frame into body frame
        let v_body = orientation.inverse_transform_vector(&v_inertial);

        let velocity = VelocityData {
            velocity_x: v_body.x,
            velocity_y: v_body.y,
            velocity_z: v_body.z,
            velocity_north: data.vel_north,
            velocity_east: data.vel_east,
            velocity_down: data.vel_down,
        };

        // push data
        self.pos_publisher.publish_immediate(pos_data.clone());
        self.pos_watch.send(pos_data);
        self.vel_publisher.publish_immediate(velocity.clone());
        self.vel_watch.send(velocity);
    }
}

struct TimeoutSelector {
    primary: SensorId,
    last_update: Instant,
    timeout: Duration,
}

impl TimeoutSelector {
    fn new(timeout: Duration) -> Self {
        Self {
            primary: SensorId::Gps1,
            last_update: Instant::now(),
            timeout,
        }
    }

    fn accept(&mut self, id: SensorId, ts: Instant) -> bool {
        if self.primary == id || self.last_update.elapsed() > self.timeout {
            self.primary = id;
            self.last_update = ts;
            true
        } else {
            false
        }
    }
}
