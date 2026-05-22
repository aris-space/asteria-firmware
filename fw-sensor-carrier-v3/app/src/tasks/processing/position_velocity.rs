use defmt::{info, trace};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant};
use nalgebra::Vector3;
use ublox::GpsFix;

use crate::sensors::{GNSS_0, GNSS_1, GnssId};
use crate::signals;
use crate::types::{GnssSample, Position, Pvt, Velocity};

const GNSS_SOURCE_TIMEOUT: Duration = Duration::from_millis(500);
const GNSS_SOURCE_MIN_DWELL: Duration = Duration::from_secs(5);
const GNSS_PDOP_SWITCH_MARGIN: f32 = 0.9;

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut sub0 = signals::GNSS_CHANNELS[GNSS_0.index()]
        .subscriber()
        .expect("too many subs on GNSS_CHANNELS; increase SUBS");
    let mut sub1 = signals::GNSS_CHANNELS[GNSS_1.index()]
        .subscriber()
        .expect("too many subs on GNSS_CHANNELS; increase SUBS");
    let mut orientation_recv = signals::ORIENTATION_WATCH.anon_receiver();

    let mut selector = QualitySelector::new(
        GNSS_SOURCE_TIMEOUT,
        GNSS_SOURCE_MIN_DWELL,
        GNSS_PDOP_SWITCH_MARGIN,
    );

    let pos_sender = signals::POSITION_WATCH.sender();
    let vel_sender = signals::VELOCITY_WATCH.sender();

    loop {
        let sample: GnssSample =
            match select(sub0.next_message_pure(), sub1.next_message_pure()).await {
                Either::First(s) | Either::Second(s) => s,
            };

        let quality = GnssQuality::from_pvt(&sample.pvt);
        if !selector.accept(sample.src, sample.ts, quality) {
            continue;
        }

        let data = sample.pvt;
        let pos = Position {
            ts: sample.ts,
            lat_deg: data.lat_deg,
            lon_deg: data.lon_deg,
            height_msl_m: data.height_msl,
            horizontal_accuracy_m: data.horiz_accuracy as f32 / 1000.0,
            vertical_accuracy_m: data.vert_accuracy as f32 / 1000.0,
        };

        let orientation = orientation_recv
            .try_get()
            .map(|o| o.q)
            .unwrap_or_else(nalgebra::UnitQuaternion::identity);
        let v_inertial = Vector3::new(data.vel_north, data.vel_east, data.vel_down);
        let v_body = orientation.inverse_transform_vector(&v_inertial);

        let vel = Velocity {
            ts: sample.ts,
            body_x: v_body.x,
            body_y: v_body.y,
            body_z: v_body.z,
            ned_north: data.vel_north,
            ned_east: data.vel_east,
            ned_down: data.vel_down,
        };

        pos_sender.send(pos);
        vel_sender.send(vel);
        trace!("pv: lat={} lon={}", data.lat_deg, data.lon_deg);
    }
}

#[derive(Clone, Copy)]
struct GnssQuality {
    fix_tier: u8,
    pdop: u16,
}

impl GnssQuality {
    fn from_pvt(data: &Pvt) -> Self {
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

struct QualitySelector {
    primary: GnssId,
    last_switch: Instant,
    gps0: Option<SourceState>,
    gps1: Option<SourceState>,
    timeout: Duration,
    min_dwell: Duration,
    pdop_margin: f32,
}

impl QualitySelector {
    fn new(timeout: Duration, min_dwell: Duration, pdop_margin: f32) -> Self {
        Self {
            primary: GNSS_0,
            last_switch: Instant::now(),
            gps0: None,
            gps1: None,
            timeout,
            min_dwell,
            pdop_margin,
        }
    }

    fn state(&self, id: GnssId) -> Option<&SourceState> {
        if id == GNSS_0 {
            self.gps0.as_ref()
        } else {
            self.gps1.as_ref()
        }
    }

    fn fresh_state(&self, id: GnssId) -> Option<&SourceState> {
        self.state(id)
            .filter(|s| s.last_update.elapsed() <= self.timeout)
    }

    fn accept(&mut self, id: GnssId, ts: Instant, quality: GnssQuality) -> bool {
        let new_state = SourceState {
            last_update: ts,
            quality,
        };
        if id == GNSS_0 {
            self.gps0 = Some(new_state);
        } else {
            self.gps1 = Some(new_state);
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
        let other = if self.primary == GNSS_0 {
            GNSS_1
        } else {
            GNSS_0
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
