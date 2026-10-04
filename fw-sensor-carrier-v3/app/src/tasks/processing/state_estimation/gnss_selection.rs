//! Chooses which GNSS receiver aids the estimator.

use embassy_time::{Duration, Instant};
use fw_sensor_carrier_v3::sef::{gnss_height_std_m, weaker_gnss_disagreement_floor_m};

use crate::sensors::{GNSS_COUNT, GnssId};
use crate::types::GnssSample;

const GNSS_FRESH: Duration = Duration::from_millis(500);
const GNSS_SWITCH_IMPROVEMENT: f32 = 1.5;

#[derive(Default)]
pub struct GnssSelection {
    latest: [Option<GnssSample>; GNSS_COUNT],
    // Kept through a receiver dropout so the weaker receiver stays penalised.
    disagreement_floor_m: [f32; GNSS_COUNT],
    selected: Option<GnssId>,
}

impl GnssSelection {
    pub fn selected(&self) -> Option<GnssId> {
        self.selected
    }

    /// Records `sample` and returns whether its receiver is the one to fuse.
    pub fn select(&mut self, sample: GnssSample) -> bool {
        // GNSS uncertainty determines weight; only an unusable solution is
        // removed here. A weak 3D fix can still provide an absolute anchor.
        let usable = sample.pvt.height_msl.is_finite()
            && sample.pvt.vel_down.is_finite()
            && matches!(
                sample.pvt.fix_type,
                ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning
            );
        self.latest[sample.src.index()] = usable.then_some(sample);
        if !usable {
            return false;
        }

        let now = Instant::now();
        let fresh = self
            .latest
            .map(|latest| latest.filter(|s| now.saturating_duration_since(s.ts) <= GNSS_FRESH));
        if let [Some(first), Some(second)] = fresh {
            // Only the less precise receiver inherits disagreement with the
            // other fix. This is an uncertainty adjustment, not a height offset.
            self.disagreement_floor_m = weaker_gnss_disagreement_floor_m(
                [first.pvt.height_msl, second.pvt.height_msl],
                [first.pvt.vel_down, second.pvt.vel_down],
                [first.ts.as_micros(), second.ts.as_micros()],
                [
                    receiver_height_std_m(&first),
                    receiver_height_std_m(&second),
                ],
            );
        }

        let Some(candidate) = fresh.into_iter().flatten().reduce(|a, b| {
            if receiver_height_std_m(&a) < receiver_height_std_m(&b) {
                a
            } else {
                b
            }
        }) else {
            return false;
        };
        // Keep the current fresh receiver until another is clearly better.
        let best = match self.selected.and_then(|id| fresh[id.index()]) {
            Some(current)
                if receiver_height_std_m(&candidate) * GNSS_SWITCH_IMPROVEMENT
                    >= receiver_height_std_m(&current) =>
            {
                current
            }
            _ => candidate,
        };
        if best.src != sample.src {
            return false;
        }
        self.selected = Some(best.src);
        true
    }

    /// Height uncertainty for `sample`, including any disagreement floor.
    pub fn height_std_m(&self, sample: &GnssSample) -> f32 {
        receiver_height_std_m(sample).max(self.disagreement_floor_m[sample.src.index()])
    }
}

fn receiver_height_std_m(sample: &GnssSample) -> f32 {
    gnss_height_std_m(sample.pvt.vert_accuracy, sample.pvt.pdop)
}
