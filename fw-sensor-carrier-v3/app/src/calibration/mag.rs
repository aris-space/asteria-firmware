use core::fmt;

use defmt::{Debug2Format, Display2Format, info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, with_timeout};
use magcal::{Solver, SolverTier};
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use crate::sensors::{MAG_BUS_1, MAG_BUS_2, MAGNETOMETER_COUNT, MagnetometerId};
use crate::signals::RAW_MAG_CHANNELS;
use crate::storage::{self, Storage};
use crate::types::{MagSample, RawMagSample};

/// Hard- and soft-iron correction, applied as `soft_iron * (board - hard_iron)`
/// in nT. The soft-iron matrix also absorbs any residual mounting rotation.
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct MagCalWire {
    pub hard_iron: [f32; 3],
    pub soft_iron: [f32; 9],
}

const NAME_LEN: usize = 16;

/// What's persisted per sensor: the applied correction plus metadata about the
/// fit that produced it, so a stored cal can be inspected later (`cal show`).
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct StoredCal {
    /// Short user label given at `cal mag <name>`, zero-padded.
    pub name: [u8; NAME_LEN],
    pub field_nt: f32,
    pub fit_error_pc: f32,
    pub samples: u16,
    pub wire: MagCalWire,
}

const fn name_bytes(s: &str) -> [u8; NAME_LEN] {
    let b = s.as_bytes();
    let mut out = [0u8; NAME_LEN];
    let mut i = 0;
    while i < b.len() && i < NAME_LEN {
        out[i] = b[i];
        i += 1;
    }
    out
}

fn name_str(b: &[u8; NAME_LEN]) -> &str {
    let len = b.iter().position(|&c| c == 0).unwrap_or(NAME_LEN);
    core::str::from_utf8(&b[..len]).unwrap_or("?")
}

impl StoredCal {
    pub fn name_str(&self) -> &str {
        name_str(&self.name)
    }
}

impl fmt::Display for StoredCal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let h = self.wire.hard_iron;
        let s = self.wire.soft_iron;
        // samples == 0 marks the built-in default (no fit ran), so its
        // field/error are meaningless; don't print them as if measured.
        if self.samples == 0 {
            write!(
                f,
                "\"{}\" (built-in default, not calibrated)",
                name_str(&self.name)
            )?;
        } else {
            write!(
                f,
                "\"{}\"  field {:.1} uT  error {:.2} %  ({} samples)",
                name_str(&self.name),
                self.field_nt / 1000.0,
                self.fit_error_pc,
                self.samples,
            )?;
        }
        write!(
            f,
            "\n  axis  hard uT   soft-iron\n   x   [{:6.1} ]  [{:8.3}{:8.3}{:8.3} ]\n   y   [{:6.1} ]  [{:8.3}{:8.3}{:8.3} ]\n   z   [{:6.1} ]  [{:8.3}{:8.3}{:8.3} ]",
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

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

/// Zero hard-iron, identity soft-iron: live samples pass through uncorrected
/// (bar the sensor-to-board negation in apply_calibration) until a real cal is
/// stored.
const IDENTITY: MagCalWire = MagCalWire {
    hard_iron: [0.0, 0.0, 0.0],
    soft_iron: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
};

/// Built-in fallback cal, labelled "default", used until a real cal is stored.
const fn default_cal(wire: MagCalWire) -> StoredCal {
    StoredCal {
        name: name_bytes("default"),
        field_nt: 0.0,
        fit_error_pc: 0.0,
        samples: 0,
        wire,
    }
}
const DEFAULTS: [StoredCal; MAGNETOMETER_COUNT] = [default_cal(IDENTITY), default_cal(IDENTITY)];
const KEYS: [storage::Key; MAGNETOMETER_COUNT] = [storage::key("mag0"), storage::key("mag1")];

/// Live per-sensor cal, written once at startup. A fresh `run` persists to
/// flash but does not touch this; a reset reloads and applies it.
static CAL: OnceLock<[StoredCal; MAGNETOMETER_COUNT]> = OnceLock::new();

/// Read each sensor's stored cal (or default) and publish it for the readout
/// to apply. Call once at startup, before the readout tasks run.
pub async fn load(storage: &Storage) {
    let cal = [
        load_one(storage, &KEYS[0], MAG_BUS_1).await,
        load_one(storage, &KEYS[1], MAG_BUS_2).await,
    ];
    let _ = CAL.init(cal);
}

/// Load one sensor's cal, logging whether it came from flash or fell back to
/// the built-in default.
async fn load_one(storage: &Storage, key: &storage::Key, id: MagnetometerId) -> StoredCal {
    match storage.load::<StoredCal>(key).await {
        Some(cal) => {
            info!("{}: cal loaded from flash: {}", id, Display2Format(&cal));
            cal
        }
        None => {
            let cal = DEFAULTS[id.index()];
            info!("{}: no cal in flash, using {}", id, Display2Format(&cal));
            cal
        }
    }
}

/// The cal applied to live samples (the one loaded at boot), per sensor.
pub fn applied() -> [StoredCal; MAGNETOMETER_COUNT] {
    *CAL.try_get().unwrap_or(&DEFAULTS)
}

/// Read the stored cal for each sensor straight from flash, for inspection.
/// This reflects what's persisted now (including a just-run cal not yet
/// applied), independent of the live values loaded at boot.
pub async fn stored(storage: &Storage) -> [Option<StoredCal>; MAGNETOMETER_COUNT] {
    [
        storage.load::<StoredCal>(&KEYS[0]).await,
        storage.load::<StoredCal>(&KEYS[1]).await,
    ]
}

/// Turn a raw mag sample (nT, sensor frame) into a calibrated, board-frame
/// `MagSample` (nT). The sensor-to-board remap on this board is a negation
/// of all three axes; the soft-iron matrix carries the iron correction and
/// any residual mounting rotation.
pub fn apply_calibration(raw: RawMagSample) -> MagSample {
    let cal = CAL.try_get().unwrap_or(&DEFAULTS)[raw.src.index()].wire;
    let hard_iron = Vector3::from(cal.hard_iron);
    let soft_iron = Matrix3::from_row_slice(&cal.soft_iron);
    let board = -Vector3::new(raw.x, raw.y, raw.z);
    let corrected = soft_iron * (board - hard_iron);
    MagSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        x: corrected.x,
        y: corrected.y,
        z: corrected.z,
    }
}

const COLLECT_WINDOW: Duration = Duration::from_secs(30);
// deci-uT keeps the ~50 uT field well inside i16; results scale back to nT.
const NT_TO_DECI_UT: f32 = 1e-2;
const DECI_UT_TO_NT: f32 = 100.0;
const MIN_VALID_NT: f32 = 22_000.0;
const MAX_VALID_NT: f32 = 67_000.0;

pub struct Fit {
    pub tier: SolverTier,
    pub field_nt: f32,
    pub fit_error_pc: f32,
    pub wire: MagCalWire,
}

pub struct CalReport {
    pub id: MagnetometerId,
    pub samples: usize,
    pub outcome: CalOutcome,
}

pub enum CalOutcome {
    /// Fit accepted and written to flash.
    Stored(Fit),
    /// Fit accepted but the flash write failed.
    StoreFailed(Fit),
    /// Field strength outside the plausible band; not stored.
    ImplausibleField { tier: SolverTier, field_nt: f32 },
    /// Solver couldn't fit (too few samples).
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
                let h = fit.wire.hard_iron;
                let s = fit.wire.soft_iron;
                write!(
                    f,
                    "{name}  ({:?}, {} samples) -> {status}\n  field {:.1} uT    error {:.2} %\n  axis  hard uT   soft-iron\n   x   [{:6.1} ]  [{:8.3}{:8.3}{:8.3} ]\n   y   [{:6.1} ]  [{:8.3}{:8.3}{:8.3} ]\n   z   [{:6.1} ]  [{:8.3}{:8.3}{:8.3} ]",
                    fit.tier,
                    self.samples,
                    fit.field_nt / 1000.0,
                    fit.fit_error_pc,
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
            CalOutcome::ImplausibleField { tier, field_nt } => write!(
                f,
                "{name}: {tier:?} fit, {} samples -> implausible field {field_nt} nT, discarded",
                self.samples
            ),
            CalOutcome::TooFewSamples => write!(f, "{name}: too few samples ({})", self.samples),
        }
    }
}

/// How often [`run`] reports collection progress to its caller.
const PROGRESS_INTERVAL: Duration = Duration::from_secs(3);

/// Collect a tumble window from both magnetometers and fit each. `progress` is
/// called every few seconds with the per-sensor sample counts so the caller
/// can show liveness; the returned reports describe each sensor.
pub async fn run(
    storage: &Storage,
    name: &str,
    mut progress: impl AsyncFnMut(usize, usize),
) -> [CalReport; MAGNETOMETER_COUNT] {
    info!(
        "mag cal: collecting ({} s tumble window)",
        COLLECT_WINDOW.as_secs()
    );
    let mut solver_0 = Solver::new();
    let mut solver_1 = Solver::new();
    collect_window(COLLECT_WINDOW, &mut solver_0, &mut solver_1, &mut progress).await;

    [
        fit_store(&mut solver_0, MAG_BUS_1, &KEYS[0], name, storage).await,
        fit_store(&mut solver_1, MAG_BUS_2, &KEYS[1], name, storage).await,
    ]
}

async fn collect_window(
    window: Duration,
    solver_0: &mut Solver,
    solver_1: &mut Solver,
    progress: &mut impl AsyncFnMut(usize, usize),
) {
    let mut sub_0 = RAW_MAG_CHANNELS[MAG_BUS_1.index()]
        .subscriber()
        .expect("mag cal: raw mag 0 subscribe failed");
    let mut sub_1 = RAW_MAG_CHANNELS[MAG_BUS_2.index()]
        .subscriber()
        .expect("mag cal: raw mag 1 subscribe failed");

    let deadline = Instant::now() + window;
    let mut next_tick = Instant::now() + PROGRESS_INTERVAL;
    while Instant::now() < deadline {
        let remaining = deadline - Instant::now();
        let next = select(sub_0.next_message_pure(), sub_1.next_message_pure());
        match with_timeout(remaining, next).await {
            Ok(Either::First(s)) => solver_0.push_sample(to_board_counts(s.x, s.y, s.z)),
            Ok(Either::Second(s)) => solver_1.push_sample(to_board_counts(s.x, s.y, s.z)),
            Err(_) => break,
        }
        if Instant::now() >= next_tick {
            progress(solver_0.sample_count(), solver_1.sample_count()).await;
            next_tick += PROGRESS_INTERVAL;
        }
    }
}

// Negation is the sensor-to-board remap (see apply_calibration); fit in board frame.
fn to_board_counts(x: f32, y: f32, z: f32) -> [i16; 3] {
    [x, y, z].map(|v| (-v * NT_TO_DECI_UT) as i16)
}

async fn fit_store(
    solver: &mut Solver,
    id: MagnetometerId,
    key: &storage::Key,
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

    let s = cal.soft_iron;
    let wire = MagCalWire {
        hard_iron: cal.hard_iron.map(|h| h * DECI_UT_TO_NT),
        soft_iron: [
            s[0][0], s[0][1], s[0][2], s[1][0], s[1][1], s[1][2], s[2][0], s[2][1], s[2][2],
        ],
    };
    let record = StoredCal {
        name: name_bytes(name),
        field_nt,
        fit_error_pc: cal.fit_error_percent,
        samples: samples.min(u16::MAX as usize) as u16,
        wire,
    };
    let fit = Fit {
        tier: cal.tier,
        field_nt,
        fit_error_pc: cal.fit_error_percent,
        wire,
    };
    let outcome = if storage.store(key, &record).await {
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
