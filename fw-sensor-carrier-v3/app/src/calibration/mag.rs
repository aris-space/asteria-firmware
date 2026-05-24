use core::fmt;

use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, with_timeout};
use magcal::{Solver, SolverTier};
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use super::Name;
use crate::sensors::{MAG_BUS_1, MAG_BUS_2, MAGNETOMETER_COUNT, MagnetometerId};
use crate::signals::RAW_MAG_CHANNELS;
use crate::storage::{self, Storage};
use crate::types::{MagSample, RawMagSample};

pub fn apply_calibration(raw: RawMagSample) -> MagSample {
    let cal = CAL.try_get().unwrap_or(&DEFAULTS)[raw.src.index()].correction;
    let board = sensor_to_board(Vector3::new(raw.x, raw.y, raw.z));
    let corrected = cal.correct_board_field(board);
    MagSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        x: corrected.x,
        y: corrected.y,
        z: corrected.z,
    }
}

pub async fn load(storage: &Storage) {
    let cal = [
        load_one(storage, MAG_BUS_1).await,
        load_one(storage, MAG_BUS_2).await,
    ];
    let _ = CAL.init(cal);
}

pub fn applied() -> [StoredCal; MAGNETOMETER_COUNT] {
    *CAL.try_get().unwrap_or(&DEFAULTS)
}

pub async fn stored(storage: &Storage) -> [Option<StoredCal>; MAGNETOMETER_COUNT] {
    [
        storage.load::<StoredCal>(&KEYS[0]).await,
        storage.load::<StoredCal>(&KEYS[1]).await,
    ]
}

/// Live per-sensor cal, written once at startup; a reset reloads and applies it.
static CAL: OnceLock<[StoredCal; MAGNETOMETER_COUNT]> = OnceLock::new();

const KEYS: [storage::Key; MAGNETOMETER_COUNT] = [
    storage::key(MAG_BUS_1.name()),
    storage::key(MAG_BUS_2.name()),
];
const DEFAULTS: [StoredCal; MAGNETOMETER_COUNT] = [StoredCal::DEFAULT; MAGNETOMETER_COUNT];

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

async fn load_one(storage: &Storage, id: MagnetometerId) -> StoredCal {
    match storage.load::<StoredCal>(&KEYS[id.index()]).await {
        Some(cal) => {
            info!("{}: cal \"{}\" loaded from flash", id, cal.name.as_str());
            cal
        }
        None => {
            info!("{}: no cal in flash, using identity (default)", id);
            StoredCal::DEFAULT
        }
    }
}

// deci-uT keeps the ~50 uT field well inside i16; results scale back to nT.
const NT_TO_DECI_UT: f32 = 1e-2;
const DECI_UT_TO_NT: f32 = 100.0;

/// What's persisted per sensor: the applied correction plus metadata about the
/// fit that produced it, so a stored cal can be inspected later (`cal show`).
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct StoredCal {
    pub name: Name,
    field_nt: f32,
    fit_error_pc: f32,
    samples: u16,
    correction: Correction,
}

impl StoredCal {
    const DEFAULT: Self = Self {
        name: Name::new("default"),
        field_nt: 0.0,
        fit_error_pc: 0.0,
        samples: 0,
        correction: Correction::IDENTITY,
    };

    fn from_fit(name: Name, fit: &Fit, samples: usize) -> Self {
        Self {
            name,
            field_nt: fit.field_nt,
            fit_error_pc: fit.fit_error_pc,
            samples: samples.min(u16::MAX as usize) as u16,
            correction: fit.correction,
        }
    }

    fn is_default(&self) -> bool {
        *self == Self::DEFAULT
    }

    /// Whether applying this stored cal would change the live one: only the
    /// label and correction are applied, so fit metadata is ignored.
    pub fn differs_from(&self, applied: &Self) -> bool {
        self.name != applied.name || self.correction != applied.correction
    }
}

impl fmt::Display for StoredCal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.is_default() {
            write!(
                f,
                "\"{}\" \x1b[31m(built-in default, not calibrated)\x1b[0m",
                self.name
            )?;
        } else {
            write!(
                f,
                "\"{}\"  field {:.1} uT  error {:.2} %  ({} samples)",
                self.name,
                self.field_nt / 1000.0,
                self.fit_error_pc,
                self.samples,
            )?;
        }
        write!(f, "\n{}", self.correction)
    }
}

/// Hard- and soft-iron correction, applied as `soft_iron * (board - hard_iron)`
/// in nT. The soft-iron matrix also absorbs any residual mounting rotation.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Correction {
    hard_iron: [f32; 3],
    soft_iron: [f32; 9],
}

impl Correction {
    const IDENTITY: Self = Self {
        hard_iron: [0.0, 0.0, 0.0],
        soft_iron: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    };

    fn from_solver(hard_iron: [f32; 3], soft_iron: [[f32; 3]; 3]) -> Self {
        Self {
            hard_iron: hard_iron.map(|h| h * DECI_UT_TO_NT),
            soft_iron: [
                soft_iron[0][0],
                soft_iron[0][1],
                soft_iron[0][2],
                soft_iron[1][0],
                soft_iron[1][1],
                soft_iron[1][2],
                soft_iron[2][0],
                soft_iron[2][1],
                soft_iron[2][2],
            ],
        }
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

const TICK: Duration = Duration::from_secs(3);
pub const PROGRESS_TICKS: usize = 10;
// Plausible Earth-field magnitude band; fits outside it are discarded.
const MIN_VALID_NT: f32 = 22_000.0;
const MAX_VALID_NT: f32 = 67_000.0;

/// Magnetometer calibration: the caller drives the collection loop (`collect_tick`
/// per window, then `finish`). One `magcal` solver per sensor.
pub struct MagCal {
    solvers: [Solver; MAGNETOMETER_COUNT],
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
            .expect("mag cal: raw mag 0 subscribe failed");
        let mut sub_1 = RAW_MAG_CHANNELS[MAG_BUS_2.index()]
            .subscriber()
            .expect("mag cal: raw mag 1 subscribe failed");
        let deadline = Instant::now() + TICK;
        while Instant::now() < deadline {
            let remaining = deadline - Instant::now();
            let next = select(sub_0.next_message_pure(), sub_1.next_message_pure());
            match with_timeout(remaining, next).await {
                Ok(Either::First(s)) => self.solvers[0].push_sample(to_board_counts(s)),
                Ok(Either::Second(s)) => self.solvers[1].push_sample(to_board_counts(s)),
                Err(_) => break,
            }
        }
    }

    pub fn counts(&self) -> [usize; MAGNETOMETER_COUNT] {
        [
            self.solvers[0].sample_count(),
            self.solvers[1].sample_count(),
        ]
    }

    pub async fn finish(self, name: &str, storage: &Storage) -> [CalReport; MAGNETOMETER_COUNT] {
        let [mut s0, mut s1] = self.solvers;
        [
            Self::finish_one(&mut s0, MAG_BUS_1, name, storage).await,
            Self::finish_one(&mut s1, MAG_BUS_2, name, storage).await,
        ]
    }

    async fn finish_one(
        solver: &mut Solver,
        id: MagnetometerId,
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

        let field_nt = cal.field_strength * DECI_UT_TO_NT;
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

        let correction = Correction::from_solver(cal.hard_iron, cal.soft_iron);
        let fit = Fit {
            tier: cal.tier,
            field_nt,
            fit_error_pc: cal.fit_error_percent,
            correction,
        };
        let record = StoredCal::from_fit(Name::new(name), &fit, samples);
        let outcome = if storage.store(&KEYS[id.index()], &record).await {
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
    field_nt: f32,
    fit_error_pc: f32,
    correction: Correction,
}

pub struct CalReport {
    id: MagnetometerId,
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
                write!(
                    f,
                    "{name}  ({:?}, {} samples) -> {status}\n  field {:.1} uT    error {:.2} %\n{}",
                    fit.tier,
                    self.samples,
                    fit.field_nt / 1000.0,
                    fit.fit_error_pc,
                    fit.correction,
                )
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

/// Sensor-to-board axis remap on this board: negate all three axes.
fn sensor_to_board(v: Vector3<f32>) -> Vector3<f32> {
    -v
}

/// Raw sample to the solver's board-frame deci-uT counts: negate all axes
/// (sensor-to-board) and scale nT to deci-uT to keep the field inside `i16`.
fn to_board_counts(s: RawMagSample) -> [i16; 3] {
    [s.x, s.y, s.z].map(|v| (-v * NT_TO_DECI_UT) as i16)
}
