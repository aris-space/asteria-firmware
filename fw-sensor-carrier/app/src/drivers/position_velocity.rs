#![allow(dead_code)]
use crate::drivers::inertial::ORIENTATION_WATCH;
use crate::sensors::SensorId;
use crate::sensors::gnss::PvtData;
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    mutex::Mutex,
    once_lock::OnceLock,
    pubsub::{ImmediatePublisher, PubSubChannel},
    watch::{Sender, Watch},
};
use embassy_time::{Duration, Instant};
use embedded_utils::{error, info};
use hermes_can::messages::sensor_data::{PositionData, VelocityData};
use hermes_can::messages::system_management::UTCTimeUpdate;
use nalgebra::{UnitQuaternion, Vector3};
use ublox::GpsFix;

pub const CAP: usize = 10;
pub const PUB: usize = 1;
pub const SUB: usize = 2;
pub const WATCH: usize = 3;

const GNSS_SOURCE_TIMEOUT: Duration = Duration::from_secs(2);
const GNSS_SOURCE_MIN_DWELL: Duration = Duration::from_secs(1);
const GNSS_PDOP_SWITCH_MARGIN: f32 = 0.9;

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

    shared: Mutex<CriticalSectionRawMutex, QualitySelector>,
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
            shared: Mutex::new(QualitySelector::new(
                GNSS_SOURCE_TIMEOUT,
                GNSS_SOURCE_MIN_DWELL,
                GNSS_PDOP_SWITCH_MARGIN,
            )),
        }
    }

    pub async fn update(&self, data: PvtData, ts: Instant, sensor_id: SensorId) {
        if !matches!(sensor_id, SensorId::Gps1 | SensorId::Gps2) {
            error!("Invalid sensor id for position update: {:?}", sensor_id);
            return;
        }

        let quality = GnssQuality::from_pvt(&data);
        if !self.shared.lock().await.accept(sensor_id, ts, quality) {
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

#[derive(Clone, Copy)]
struct GnssQuality {
    /// Higher = better fix. 3 = 3D (incl. 3D+DR), 2 = 2D, 1 = DR-only, 0 = none/other
    fix_tier: u8,
    /// u-blox PDOP, scale 0.01. Lower is better. 0 means "no valid DOP".
    pdop: u16,
}

impl GnssQuality {
    fn from_pvt(data: &PvtData) -> Self {
        let fix_tier = match data.fix_type {
            GpsFix::Fix3D | GpsFix::GPSPlusDeadReckoning => 3,
            GpsFix::Fix2D => 2,
            GpsFix::DeadReckoningOnly => 1,
            _ => 0,
        };
        Self {
            fix_tier,
            pdop: data.pdop,
        }
    }
}

#[derive(Clone, Copy)]
struct SourceState {
    last_update: Instant,
    quality: GnssQuality,
}

/// Picks the best available GNSS source per sample.
///
/// Switching rules, in priority order:
/// 1. **Liveness**: a primary that hasn't reported within `timeout` loses to
///    any fresh source. Immediate, not rate-limited.
/// 2. **Fix tier** (3D > 2D > DR > none): a strictly higher tier wins outright.
///    Immediate, not rate-limited
/// 3. **PDOP** within the same tier: a candidate takes over only if both (a)
///    `candidate.pdop < primary.pdop * pdop_margin` (better by a margin) and (b)
///    the current primary has held for at least `min_dwell`
struct QualitySelector {
    primary: SensorId,
    last_switch: Instant,
    gps1: Option<SourceState>,
    gps2: Option<SourceState>,
    timeout: Duration,
    min_dwell: Duration,
    /// Candidate's PDOP must be < `primary.pdop * pdop_margin` to qualify.
    pdop_margin: f32,
}

impl QualitySelector {
    fn new(timeout: Duration, min_dwell: Duration, pdop_margin: f32) -> Self {
        Self {
            primary: SensorId::Gps1,
            last_switch: Instant::now(),
            gps1: None,
            gps2: None,
            timeout,
            min_dwell,
            pdop_margin,
        }
    }

    fn state(&self, id: SensorId) -> Option<&SourceState> {
        match id {
            SensorId::Gps1 => self.gps1.as_ref(),
            SensorId::Gps2 => self.gps2.as_ref(),
            _ => None,
        }
    }

    fn fresh_state(&self, id: SensorId) -> Option<&SourceState> {
        self.state(id)
            .filter(|s| s.last_update.elapsed() <= self.timeout)
    }

    fn accept(&mut self, id: SensorId, ts: Instant, quality: GnssQuality) -> bool {
        let new_state = SourceState {
            last_update: ts,
            quality,
        };
        match id {
            SensorId::Gps1 => self.gps1 = Some(new_state),
            SensorId::Gps2 => self.gps2 = Some(new_state),
            _ => return false,
        }

        let previous = self.primary;
        self.choose_primary();
        if self.primary != previous {
            self.last_switch = Instant::now();
            let q = self.state(self.primary).map(|s| s.quality);
            info!(
                "GNSS primary {:?} -> {:?} (fix_tier={}, pdop={})",
                previous,
                self.primary,
                q.map(|q| q.fix_tier).unwrap_or(0),
                q.map(|q| q.pdop).unwrap_or(0),
            );
        }

        self.primary == id
    }

    fn choose_primary(&mut self) {
        let other = match self.primary {
            SensorId::Gps1 => SensorId::Gps2,
            SensorId::Gps2 => SensorId::Gps1,
            _ => return,
        };

        let Some(p) = self.fresh_state(self.primary) else {
            if self.fresh_state(other).is_some() {
                self.primary = other;
            }
            return;
        };
        let Some(o) = self.fresh_state(other) else {
            return;
        };

        if o.quality.fix_tier > p.quality.fix_tier {
            self.primary = other;
            return;
        }
        if o.quality.fix_tier < p.quality.fix_tier {
            return;
        }

        let dwell_elapsed = self.last_switch.elapsed() >= self.min_dwell;
        match (p.quality.pdop, o.quality.pdop) {
            (0, o_pdop) if o_pdop > 0 => self.primary = other,
            (_, 0) => {}
            (p_pdop, o_pdop) => {
                if (o_pdop as f32) < (p_pdop as f32) * self.pdop_margin && dwell_elapsed {
                    self.primary = other;
                }
            }
        }
    }
}
