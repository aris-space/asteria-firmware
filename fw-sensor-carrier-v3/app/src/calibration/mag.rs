//! Magnetometer calibration: latency, LSM303AGR units, axes, and hard- and
//! soft-iron correction.

use core::fmt;

use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, with_timeout};
use magcal::{Solver, SolverTier};
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use super::{Calibrations, Name};
use crate::sensors::{MAG_BUS_1, MAG_BUS_2, MAG_COUNT, MagId};
use crate::signals::RAW_MAG_CHANNELS;
use crate::storage::Storage;
use crate::types::{MagSample, RawMagSample};

// The LSM303AGR reports 150 nT per count; the solver fits in native counts and
// results scale back to nT.
const LSB_TO_NT: f32 = 150.0;

/// Sensor-to-board axis remap on this board: negate all three axes, in the
/// magnetometer's native counts (so the cal solver fits at full resolution).
fn sensor_to_board(counts: [i16; 3]) -> [i16; 3] {
    counts.map(i16::saturating_neg)
}

/// Hard- and soft-iron correction, applied as `soft_iron * (board - hard_iron)`
/// in nT, with the fit that produced it. The soft-iron matrix also absorbs any
/// residual mounting rotation.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Correction {
    field_nt: f32,
    fit_error_pc: f32,
    samples: u16,
    hard_iron: [f32; 3],
    soft_iron: [f32; 9],
}

impl super::Correction for Correction {
    const DEFAULT: Self = Self {
        field_nt: 0.0,
        fit_error_pc: 0.0,
        samples: 0,
        hard_iron: [0.0, 0.0, 0.0],
        soft_iron: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    };
}

impl Correction {
    fn from_fit(cal: &magcal::MagCal, samples: usize) -> Self {
        let s = cal.soft_iron;
        Self {
            field_nt: cal.field_strength * LSB_TO_NT,
            fit_error_pc: cal.fit_error_percent,
            samples: samples.min(u16::MAX as usize) as u16,
            hard_iron: cal.hard_iron.map(|h| h * LSB_TO_NT),
            soft_iron: [
                s[0][0], s[0][1], s[0][2], s[1][0], s[1][1], s[1][2], s[2][0], s[2][1], s[2][2],
            ],
        }
    }

    /// An identity fallback has no measured hard- or soft-iron correction.
    pub fn is_calibrated(&self) -> bool {
        *self != <Self as super::Correction>::DEFAULT
    }

    /// Accept only calibrated samples with a plausible Earth-field magnitude.
    pub fn accepts_field(&self, field_nt: f32) -> bool {
        self.is_calibrated() && (MIN_VALID_NT..=MAX_VALID_NT).contains(&field_nt)
    }

    fn correct_board_field(&self, board: Vector3<f32>) -> Vector3<f32> {
        let hard_iron = Vector3::from(self.hard_iron);
        let soft_iron = Matrix3::from_row_slice(&self.soft_iron);
        soft_iron * (board - hard_iron)
    }
}

impl fmt::Display for Correction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let h = self.hard_iron;
        let s = self.soft_iron;
        writeln!(
            f,
            "  field {:.1} uT  error {:.2} %  ({} samples)",
            self.field_nt / 1000.0,
            self.fit_error_pc,
            self.samples,
        )?;
        writeln!(f, "  axis  hard uT   {:^27}", "soft-iron")?;
        write!(
            f,
            "   x   [{:6.1} ]  [{:8.3}{:8.3}{:8.3} ]\n   y   [{:6.1} ]  [{:8.3}{:8.3}{:8.3} ]\n   z   [{:6.1} ]  [{:8.3}{:8.3}{:8.3} ]",
            h[0] / 1000.0,
            s[0],
            s[1],
            s[2],
            h[1] / 1000.0,
            s[3],
            s[4],
            s[5],
            h[2] / 1000.0,
            s[6],
            s[7],
            s[8],
        )
    }
}

pub static CAL: Calibrations<MagId, Correction, MAG_COUNT> = Calibrations::new(MagId::ALL);

pub fn apply_calibration(raw: RawMagSample) -> MagSample {
    let cal = CAL.applied(raw.src);
    let board = sensor_to_board([raw.x, raw.y, raw.z]).map(|c| c as f32 * LSB_TO_NT);
    let corrected = cal.correction.correct_board_field(Vector3::from(board));
    MagSample {
        src: raw.src,
        ts: cal.sample_time(raw.ts),
        x: corrected.x,
        y: corrected.y,
        z: corrected.z,
    }
}

const TICK: Duration = Duration::from_secs(3);
pub const PROGRESS_TICKS: usize = 10;
// Plausible Earth-field magnitude band; fits outside it are discarded.
const MIN_VALID_NT: f32 = 22_000.0;
const MAX_VALID_NT: f32 = 67_000.0;

const _: () = assert!(
    MAG_COUNT == 2,
    "MagCal is written for exactly two magnetometers"
);

/// Magnetometer calibration: the caller runs the collection loop (`collect_tick`
/// per window, then `finish`). One `magcal` solver per sensor.
pub struct MagCal {
    solvers: [Solver; MAG_COUNT],
}

impl Default for MagCal {
    fn default() -> Self {
        Self {
            solvers: [Solver::new(), Solver::new()],
        }
    }
}

impl MagCal {
    pub async fn collect_tick(&mut self) {
        let mut sub_0 = RAW_MAG_CHANNELS[MAG_BUS_1.index()]
            .subscriber()
            .expect("too many subs on RAW_MAG_CHANNELS; increase SUBS");
        let mut sub_1 = RAW_MAG_CHANNELS[MAG_BUS_2.index()]
            .subscriber()
            .expect("too many subs on RAW_MAG_CHANNELS; increase SUBS");
        let deadline = Instant::now() + TICK;
        while Instant::now() < deadline {
            let remaining = deadline - Instant::now();
            let next = select(sub_0.next_message_pure(), sub_1.next_message_pure());
            match with_timeout(remaining, next).await {
                Ok(Either::First(s)) => {
                    self.solvers[0].push_sample(sensor_to_board([s.x, s.y, s.z]))
                }
                Ok(Either::Second(s)) => {
                    self.solvers[1].push_sample(sensor_to_board([s.x, s.y, s.z]))
                }
                Err(_) => break,
            }
        }
    }

    pub fn counts(&self) -> [usize; MAG_COUNT] {
        [
            self.solvers[0].sample_count(),
            self.solvers[1].sample_count(),
        ]
    }

    pub async fn finish(self, name: &str, storage: &Storage) -> [CalReport; MAG_COUNT] {
        let [mut s0, mut s1] = self.solvers;
        [
            Self::finish_one(&mut s0, MAG_BUS_1, name, storage).await,
            Self::finish_one(&mut s1, MAG_BUS_2, name, storage).await,
        ]
    }

    async fn finish_one(
        solver: &mut Solver,
        id: MagId,
        name: &str,
        storage: &Storage,
    ) -> CalReport {
        let samples = solver.sample_count();
        let cal = match solver.solve() {
            Ok(cal) => cal,
            Err(_) => {
                warn!("{}: fit failed, too few samples ({})", id, samples);
                return CalReport {
                    id,
                    samples,
                    outcome: CalOutcome::TooFewSamples,
                };
            }
        };

        let field_nt = cal.field_strength * LSB_TO_NT;
        info!(
            "{}: fit {} B={=f32} nT err={=f32} %",
            id,
            Debug2Format(&cal.tier),
            field_nt,
            cal.fit_error_percent
        );

        if !(MIN_VALID_NT..=MAX_VALID_NT).contains(&field_nt) {
            warn!("{}: implausible field {=f32} nT, discarding", id, field_nt);
            return CalReport {
                id,
                samples,
                outcome: CalOutcome::ImplausibleField {
                    tier: cal.tier,
                    field_nt,
                },
            };
        }

        let correction = Correction::from_fit(&cal, samples);
        let fit = Fit {
            tier: cal.tier,
            correction,
        };
        let outcome = if CAL
            .store_correction(storage, id, Name::new(name), correction)
            .await
        {
            CalOutcome::Stored(fit)
        } else {
            CalOutcome::StoreFailed(fit)
        };
        CalReport {
            id,
            samples,
            outcome,
        }
    }
}

struct Fit {
    tier: SolverTier,
    correction: Correction,
}

pub struct CalReport {
    id: MagId,
    samples: usize,
    outcome: CalOutcome,
}

enum CalOutcome {
    Stored(Fit),
    StoreFailed(Fit),
    /// Field strength outside the plausible band.
    ImplausibleField {
        tier: SolverTier,
        field_nt: f32,
    },
    TooFewSamples,
}

impl CalReport {
    pub fn stored(&self) -> bool {
        matches!(self.outcome, CalOutcome::Stored(_))
    }
}

impl fmt::Display for CalReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = self.id.name();
        match &self.outcome {
            CalOutcome::Stored(fit) | CalOutcome::StoreFailed(fit) => {
                let status = if self.stored() {
                    "stored"
                } else {
                    "FLASH WRITE FAILED"
                };
                write!(f, "{name}  ({:?}) -> {status}{}", fit.tier, fit.correction,)
            }
            CalOutcome::ImplausibleField { tier, field_nt } => write!(
                f,
                "{name}: {tier:?} fit, {} samples -> implausible field {field_nt} nT, discarded",
                self.samples
            ),
            CalOutcome::TooFewSamples => write!(f, "{name}: too few samples ({})", self.samples),
        }
    }
}
