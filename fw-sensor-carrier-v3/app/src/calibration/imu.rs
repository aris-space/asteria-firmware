use core::fmt;

use defmt::{info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, with_timeout};
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use super::imu_fit::{Estimator, Fit};
use crate::sensors::IMU_COUNT;
use crate::signals::RAW_IMU_CHANNELS;
use crate::storage::{self, Storage};
use crate::types::{ImuSample, RawImuSample};

/// Per-IMU readout correction. IMU 0 is the reference (identity rotation);
/// IMU 1's rotation brings it into IMU 0's frame.
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuCalWire {
    pub fine_rot: [f32; 9],
    /// Zero-rate offset in board frame, dps.
    pub gyro_bias: [f32; 3],
}

impl ImuCalWire {
    fn from_parts(rotation: Matrix3<f32>, bias: Vector3<f32>) -> Self {
        Self {
            fine_rot: row_major(&rotation),
            gyro_bias: bias.into(),
        }
    }

    fn rotation(&self) -> Matrix3<f32> {
        Matrix3::from_row_slice(&self.fine_rot)
    }

    fn bias(&self) -> Vector3<f32> {
        Vector3::from(self.gyro_bias)
    }
}

const NAME_LEN: usize = 16;

/// Persisted per IMU: the applied correction plus the shared cross-IMU fit
/// metadata (one fit covers both IMUs); only `wire` differs between them.
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct StoredCal {
    pub name: [u8; NAME_LEN],
    pub residual_deg: f32,
    pub coverage: f32,
    pub pairs: u32,
    pub wire: ImuCalWire,
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
        // pairs == 0 marks the built-in default (no fit ran).
        if self.pairs == 0 {
            writeln!(
                f,
                "\"{}\" (built-in default, not calibrated)",
                self.name_str()
            )?;
        } else {
            writeln!(
                f,
                "\"{}\"  residual {:.2} deg  coverage {:.2}  ({} pairs)",
                self.name_str(),
                self.residual_deg,
                self.coverage,
                self.pairs
            )?;
        }
        write!(f, "{}", self.wire)
    }
}

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

const IDENTITY: ImuCalWire = ImuCalWire {
    fine_rot: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    gyro_bias: [0.0, 0.0, 0.0],
};

const fn default_cal(wire: ImuCalWire) -> StoredCal {
    StoredCal {
        name: name_bytes("default"),
        residual_deg: 0.0,
        coverage: 0.0,
        pairs: 0,
        wire,
    }
}

const DEFAULTS: [StoredCal; IMU_COUNT] = [default_cal(IDENTITY), default_cal(IDENTITY)];
const KEYS: [storage::Key; IMU_COUNT] = [storage::key("imu0"), storage::key("imu1")];

/// Live per-IMU cal, written once at startup; a reset reloads and applies it.
static CAL: OnceLock<[StoredCal; IMU_COUNT]> = OnceLock::new();

pub async fn load(storage: &Storage) {
    let cal = [
        load_one(storage, &KEYS[0], 0).await,
        load_one(storage, &KEYS[1], 1).await,
    ];
    let _ = CAL.init(cal);
}

async fn load_one(storage: &Storage, key: &storage::Key, idx: usize) -> StoredCal {
    match storage.load::<StoredCal>(key).await {
        Some(cal) => {
            info!("IMU {}: cal \"{}\" loaded from flash", idx, cal.name_str());
            cal
        }
        None => {
            info!(
                "IMU {}: no cal in flash, using identity (no correction)",
                idx
            );
            default_cal(IDENTITY)
        }
    }
}

pub fn applied() -> [StoredCal; IMU_COUNT] {
    *CAL.try_get().unwrap_or(&DEFAULTS)
}

pub async fn stored(storage: &Storage) -> [Option<StoredCal>; IMU_COUNT] {
    [
        storage.load::<StoredCal>(&KEYS[0]).await,
        storage.load::<StoredCal>(&KEYS[1]).await,
    ]
}

/// Sensor-to-board axis remap for the LSM6DSO32 on this board: negate x and z.
fn sensor_to_board(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(-v.x, v.y, -v.z)
}

pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    let cal = CAL.try_get().unwrap_or(&DEFAULTS)[raw.src.index()].wire;
    let rot = cal.rotation();
    let accel = rot * sensor_to_board(Vector3::new(raw.accel.x, raw.accel.y, raw.accel.z));
    let gyro =
        rot * (sensor_to_board(Vector3::new(raw.gyro.x, raw.gyro.y, raw.gyro.z)) - cal.bias());
    ImuSample {
        src: raw.src,
        ts: raw.ts - DELAY,
        accel: lsm6dso32::Acceleration {
            x: accel.x,
            y: accel.y,
            z: accel.z,
        },
        gyro: lsm6dso32::AngularRate {
            x: gyro.x,
            y: gyro.y,
            z: gyro.z,
        },
    }
}

pub const POSES: usize = 6;
const POSE_CAPTURE: Duration = Duration::from_secs(2);
const STILL_MOTION_DPS: f32 = 30.0;
const GOOD_RESIDUAL_DEG: f32 = 2.0;
const GOOD_COVERAGE: f32 = 0.7;

pub struct ImuCalReport {
    pub fit: Fit,
    pub still_ok: bool,
    pub stored: bool,
}

impl ImuCalReport {
    pub fn stored(&self) -> bool {
        self.stored
    }
}

/// Cross-IMU calibration: the caller drives the pose loop (`capture_pose` per
/// pose, then `finish`). IMU 0 is the reference; IMU 1 rotates into its frame.
#[derive(Default)]
pub struct ImuCal {
    est: Estimator,
}

impl ImuCal {
    pub async fn capture_pose(&mut self) -> [usize; IMU_COUNT] {
        let mut sub_0 = RAW_IMU_CHANNELS[0]
            .subscriber()
            .expect("imu cal: raw imu 0 subscribe failed");
        let mut sub_1 = RAW_IMU_CHANNELS[1]
            .subscriber()
            .expect("imu cal: raw imu 1 subscribe failed");
        self.est.begin_pose();
        let mut got = [0usize; IMU_COUNT];
        let deadline = Instant::now() + POSE_CAPTURE;
        while Instant::now() < deadline {
            let remaining = deadline - Instant::now();
            let next = select(sub_0.next_message_pure(), sub_1.next_message_pure());
            let (idx, s) = match with_timeout(remaining, next).await {
                Ok(Either::First(s)) => (0, s),
                Ok(Either::Second(s)) => (1, s),
                Err(_) => break,
            };
            let accel = sensor_to_board(Vector3::new(s.accel.x, s.accel.y, s.accel.z));
            let gyro = sensor_to_board(Vector3::new(s.gyro.x, s.gyro.y, s.gyro.z));
            if self.est.observe(idx, accel, gyro) {
                got[idx] += 1;
            }
        }
        got
    }

    pub async fn finish(self, name: &str, storage: &Storage) -> ImuCalReport {
        let fit = self.est.solve();
        let still_ok = fit.peak_dps < STILL_MOTION_DPS && fit.gyro_n[0] > 0 && fit.gyro_n[1] > 0;
        if !still_ok {
            warn!(
                "imu cal: poses not still enough (peak {=f32} dps), gyro bias not stored",
                fit.peak_dps
            );
        }
        let bias = if still_ok {
            fit.gyro_bias
        } else {
            [Vector3::zeros(); IMU_COUNT]
        };
        let record = |wire| StoredCal {
            name: name_bytes(name),
            residual_deg: fit.rotation.residual_deg,
            coverage: fit.rotation.coverage,
            pairs: fit.pairs as u32,
            wire,
        };
        let cal = [
            record(ImuCalWire::from_parts(Matrix3::identity(), bias[0])),
            record(ImuCalWire::from_parts(fit.rotation.rotation, bias[1])),
        ];
        let stored =
            storage.store(&KEYS[0], &cal[0]).await && storage.store(&KEYS[1], &cal[1]).await;
        info!(
            "imu cal \"{}\" done: misalign {=f32} deg, {=usize} pairs, stored {=bool}",
            name, fit.rotation.misalign_deg, fit.pairs, stored
        );
        ImuCalReport {
            fit,
            still_ok,
            stored,
        }
    }
}

fn row_major(m: &Matrix3<f32>) -> [f32; 9] {
    [
        m[(0, 0)],
        m[(0, 1)],
        m[(0, 2)],
        m[(1, 0)],
        m[(1, 1)],
        m[(1, 2)],
        m[(2, 0)],
        m[(2, 1)],
        m[(2, 2)],
    ]
}

fn write_row(f: &mut fmt::Formatter<'_>, label: &str, x: f32, y: f32, z: f32) -> fmt::Result {
    writeln!(f, "  {label:<21}[{x:8.3}{y:8.3}{z:8.3} ]")
}

fn bias_row(
    f: &mut fmt::Formatter<'_>,
    label: &str,
    tag: &str,
    v: Vector3<f32>,
    norm: bool,
) -> fmt::Result {
    write!(f, "  {label:<21}{tag} [{:7.2}{:7.2}{:7.2} ]", v.x, v.y, v.z)?;
    if norm {
        write!(f, "  |{:.2}|", v.norm())?;
    }
    writeln!(f)
}

impl fmt::Display for ImuCalReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // \x1b[..m are ANSI colours for the console: failures red, warnings yellow.
        let status = if self.stored {
            "stored"
        } else {
            "\x1b[31mFLASH WRITE FAILED\x1b[0m"
        };
        let fit = &self.fit;
        writeln!(f, "imu cal -> {status}   {} gravity pairs\n", fit.pairs)?;

        if fit.pairs == 0 {
            writeln!(
                f,
                "  {:<21}\x1b[33mno gravity captured (hold stiller at each pose)\x1b[0m\n",
                "rotation"
            )?;
        } else {
            let rot = &fit.rotation;
            match rot.axis {
                Some(a) => writeln!(
                    f,
                    "  {:<21}{:.2} deg about [{:7.3}{:7.3}{:7.3} ]",
                    "rotation IMU1->IMU0", rot.misalign_deg, a.x, a.y, a.z
                )?,
                None => writeln!(
                    f,
                    "  {:<21}{:.2} deg (near-aligned)",
                    "rotation IMU1->IMU0", rot.misalign_deg
                )?,
            }
            writeln!(
                f,
                "  {:<21}residual {:.2} deg   coverage {:.2}",
                "", rot.residual_deg, rot.coverage
            )?;
            if rot.coverage < GOOD_COVERAGE {
                writeln!(
                    f,
                    "  {:<21}\x1b[33m(!) low coverage - add poses tilted on edge/corner (gravity sideways)\x1b[0m",
                    ""
                )?;
            }
            if rot.residual_deg > GOOD_RESIDUAL_DEG {
                writeln!(
                    f,
                    "  {:<21}\x1b[33m(!) high scatter - rest on a firm surface and hold stiller\x1b[0m",
                    ""
                )?;
            }
            let m = &rot.rotation;
            write_row(f, "", m[(0, 0)], m[(0, 1)], m[(0, 2)])?;
            write_row(f, "", m[(1, 0)], m[(1, 1)], m[(1, 2)])?;
            write_row(f, "", m[(2, 0)], m[(2, 1)], m[(2, 2)])?;
            writeln!(f)?;
        }

        let (b0, b1) = (fit.gyro_bias[0], fit.gyro_bias[1]);
        if self.still_ok {
            bias_row(f, "gyro bias dps", "IMU0", b0, true)?;
            bias_row(f, "", "IMU1", b1, true)?;
            bias_row(f, "", "diff", b1 - b0, false)?;
            writeln!(f)?;
        } else {
            writeln!(
                f,
                "  {:<21}\x1b[31mboard moved (peak {:.1} dps); not stored\x1b[0m\n",
                "gyro bias", fit.peak_dps
            )?;
        }

        let (a0, a1) = (fit.accel_g[0], fit.accel_g[1]);
        let ratio = if a0 != 0.0 { a1 / a0 } else { f32::NAN };
        writeln!(
            f,
            "  {:<21}IMU0 {a0:.3} ({:+.1}%)   IMU1 {a1:.3} ({:+.1}%)   ratio {ratio:.3}",
            "accel mag g",
            (a0 - 1.0) * 100.0,
            (a1 - 1.0) * 100.0
        )?;
        write!(
            f,
            "  {:<21}gravity {} / {}   still {}   peak {:.1} dps",
            "samples",
            fit.gravity_n[0],
            fit.gravity_n[1],
            fit.gyro_n[0] + fit.gyro_n[1],
            fit.peak_dps
        )
    }
}

impl fmt::Display for ImuCalWire {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let r = &self.fine_rot;
        let b = &self.gyro_bias;
        writeln!(f, "fine-rot / gyro bias dps")?;
        writeln!(
            f,
            "   x   [{:8.3}{:8.3}{:8.3} ]   bias [{:7.2} ]",
            r[0], r[1], r[2], b[0]
        )?;
        writeln!(
            f,
            "   y   [{:8.3}{:8.3}{:8.3} ]        [{:7.2} ]",
            r[3], r[4], r[5], b[1]
        )?;
        write!(
            f,
            "   z   [{:8.3}{:8.3}{:8.3} ]        [{:7.2} ]",
            r[6], r[7], r[8], b[2]
        )
    }
}
