use core::fmt;

use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, select};
use embassy_sync::once_lock::OnceLock;
use embassy_time::{Duration, Instant, with_timeout};
use nalgebra::{Matrix3, Rotation3, Vector3};
use serde::{Deserialize, Serialize};

use crate::sensors::IMU_COUNT;
use crate::signals::RAW_IMU_CHANNELS;
use crate::storage::{self, Storage};
use crate::types::{ImuSample, RawImuSample};

/// Per-IMU correction applied at readout: a residual rotation (sensor-to-board
/// fine alignment) and a gyro zero-rate bias subtracted before rotating. IMU 0
/// is the reference, so its rotation stays identity; IMU 1's rotation brings it
/// into IMU 0's frame.
#[derive(Clone, Copy, Serialize, Deserialize)]
pub struct ImuCalWire {
    pub fine_rot: [f32; 9],
    /// Zero-rate offset in board frame, dps.
    pub gyro_bias: [f32; 3],
}

/// How long ago (relative to read-completion time) the physical
/// measurement actually happened.
const DELAY: Duration = Duration::from_millis(0);

const IDENTITY: ImuCalWire = ImuCalWire {
    fine_rot: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    gyro_bias: [0.0, 0.0, 0.0],
};

const DEFAULTS: [ImuCalWire; IMU_COUNT] = [IDENTITY, IDENTITY];
const KEYS: [storage::Key; IMU_COUNT] = [storage::key("imu0"), storage::key("imu1")];

/// Live per-IMU cal, written once at startup. A fresh cal persists to flash
/// but does not touch this; a reset reloads and applies it.
static CAL: OnceLock<[ImuCalWire; IMU_COUNT]> = OnceLock::new();

/// Read each IMU's stored cal (or identity) and publish it for the readout to
/// apply. Call once at startup, before the readout tasks run.
pub async fn load(storage: &Storage) {
    let cal = [
        load_one(storage, &KEYS[0], 0).await,
        load_one(storage, &KEYS[1], 1).await,
    ];
    let _ = CAL.init(cal);
}

/// Load one IMU's cal, logging whether it came from flash or fell back to the
/// identity (no-correction) default.
async fn load_one(storage: &Storage, key: &storage::Key, idx: usize) -> ImuCalWire {
    match storage.load::<ImuCalWire>(key).await {
        Some(cal) => {
            info!(
                "IMU {}: cal loaded from flash, rot {} bias {} dps",
                idx,
                Debug2Format(&cal.fine_rot),
                Debug2Format(&cal.gyro_bias)
            );
            cal
        }
        None => {
            info!(
                "IMU {}: no cal in flash, using identity (no correction)",
                idx
            );
            IDENTITY
        }
    }
}

/// The cal applied to live samples (the one loaded at boot), per IMU.
pub fn applied() -> [ImuCalWire; IMU_COUNT] {
    *CAL.try_get().unwrap_or(&DEFAULTS)
}

/// Read the stored cal for each IMU straight from flash, for inspection
/// (reflects a just-run cal not yet applied), independent of the live values.
pub async fn stored(storage: &Storage) -> [Option<ImuCalWire>; IMU_COUNT] {
    [
        storage.load::<ImuCalWire>(&KEYS[0]).await,
        storage.load::<ImuCalWire>(&KEYS[1]).await,
    ]
}

/// Sensor-to-board coarse axis remap for the LSM6DSO32 on this board:
/// negate x and z, keep y.
fn sensor_to_board(v: Vector3<f32>) -> Vector3<f32> {
    Vector3::new(-v.x, v.y, -v.z)
}

pub fn apply_calibration(raw: RawImuSample) -> ImuSample {
    let cal = CAL.try_get().unwrap_or(&DEFAULTS)[raw.src.index()];
    let fine_rot = Matrix3::from_row_slice(&cal.fine_rot);
    let bias = Vector3::from(cal.gyro_bias);
    let accel = fine_rot * sensor_to_board(Vector3::new(raw.accel.x, raw.accel.y, raw.accel.z));
    let gyro =
        fine_rot * (sensor_to_board(Vector3::new(raw.gyro.x, raw.gyro.y, raw.gyro.z)) - bias);
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

const POSES: usize = 6;
const POSE_CAPTURE: Duration = Duration::from_secs(2);
// Accept a sample as gravity only when |a| is near 1 g and the gyro shows the
// board is still: while moving, the two IMUs (at different spots on the board)
// read different acceleration, which no single rotation can reconcile.
const GRAVITY_LO_G: f32 = 0.95;
const GRAVITY_HI_G: f32 = 1.05;
const QUASI_STATIC_DPS: f32 = 4.0;
const STILL_MOTION_DPS: f32 = 30.0;
const GOOD_RESIDUAL_DEG: f32 = 2.0;
const GOOD_COVERAGE: f32 = 0.7;

/// Drives the guided pose prompts for the [`run`] callback.
pub enum Phase {
    /// Ask the user to place the board in pose `index` of `total`, then block
    /// (e.g. on a keypress) before returning so capture starts when ready.
    Pose { index: usize, total: usize },
    /// One pose captured: how many still gravity samples each IMU contributed.
    Captured {
        index: usize,
        total: usize,
        n0: usize,
        n1: usize,
    },
}

pub struct ImuCalReport {
    pub pairs: usize,
    /// IMU 1 -> IMU 0 alignment: `accel0 ~= rotation * accel1`.
    pub rotation: Matrix3<f32>,
    pub misalign_deg: f32,
    pub residual_deg: f32,
    /// Smallest/largest gravity-spread singular value: 1.0 = poses covered all
    /// axes, near 0 = they stayed in roughly one plane so the rotation about the
    /// missing axis is poorly constrained.
    pub coverage: f32,
    pub gyro_bias_dps: [Vector3<f32>; IMU_COUNT],
    pub accel_g: [f32; IMU_COUNT],
    pub gravity_n: [usize; IMU_COUNT],
    pub still_n: usize,
    pub peak_dps: f32,
    pub still_ok: bool,
    pub stored: bool,
}

impl ImuCalReport {
    pub fn stored(&self) -> bool {
        self.stored
    }
}

#[derive(Default)]
struct Accum {
    // Gravity cross-covariance `sum(a1 * a0^T)` for the Kabsch rotation fit.
    h: Matrix3<f32>,
    pairs: usize,
    accel_mag_sum: [f32; IMU_COUNT],
    gravity_n: [usize; IMU_COUNT],
    gyro_sum: [Vector3<f32>; IMU_COUNT],
    gyro_n: [usize; IMU_COUNT],
    peak_dps: f32,
}

/// Run the cross-IMU calibration as a sequence of still poses. At each pose the
/// board rests in a new orientation; gravity gives the relative rotation and the
/// (still) gyro readings give each IMU's zero-rate bias. IMU 0 is the reference
/// (identity rotation); IMU 1 is rotated into its frame. `progress` prompts for
/// each pose (and should block until the user is ready) and reports each capture.
pub async fn run(storage: &Storage, mut progress: impl AsyncFnMut(Phase)) -> ImuCalReport {
    info!("imu cal: {=usize} guided poses", POSES);
    let mut acc = Accum::default();
    for index in 0..POSES {
        progress(Phase::Pose {
            index,
            total: POSES,
        })
        .await;
        let (n0, n1) = capture_pose(&mut acc).await;
        progress(Phase::Captured {
            index,
            total: POSES,
            n0,
            n1,
        })
        .await;
    }
    finalize(storage, acc).await
}

async fn capture_pose(acc: &mut Accum) -> (usize, usize) {
    let mut sub_0 = RAW_IMU_CHANNELS[0]
        .subscriber()
        .expect("imu cal: raw imu 0 subscribe failed");
    let mut sub_1 = RAW_IMU_CHANNELS[1]
        .subscriber()
        .expect("imu cal: raw imu 1 subscribe failed");
    let mut g = [None::<Vector3<f32>>; IMU_COUNT];
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
        let gyro = sensor_to_board(Vector3::new(s.gyro.x, s.gyro.y, s.gyro.z));
        acc.peak_dps = acc.peak_dps.max(gyro.norm());
        if gyro.norm() > QUASI_STATIC_DPS {
            continue;
        }
        acc.gyro_sum[idx] += gyro;
        acc.gyro_n[idx] += 1;
        let a = sensor_to_board(Vector3::new(s.accel.x, s.accel.y, s.accel.z));
        let n = a.norm();
        if !(GRAVITY_LO_G..=GRAVITY_HI_G).contains(&n) {
            continue;
        }
        let u = a / n;
        acc.accel_mag_sum[idx] += n;
        acc.gravity_n[idx] += 1;
        got[idx] += 1;
        g[idx] = Some(u);
        if let Some(other) = g[idx ^ 1] {
            // H = sum(a1 * a0^T): order the outer product by which IMU is which.
            acc.h += if idx == 0 {
                other * u.transpose()
            } else {
                u * other.transpose()
            };
            acc.pairs += 1;
        }
    }
    (got[0], got[1])
}

async fn finalize(storage: &Storage, acc: Accum) -> ImuCalReport {
    let (rotation, misalign_deg, residual_deg, coverage) = solve_rotation(acc.h, acc.pairs);
    let still_ok = acc.peak_dps < STILL_MOTION_DPS && acc.gyro_n[0] > 0 && acc.gyro_n[1] > 0;
    if !still_ok {
        warn!(
            "imu cal: poses not still enough (peak {=f32} dps), gyro bias not stored",
            acc.peak_dps
        );
    }
    let gyro_bias_dps = if still_ok {
        [
            mean(acc.gyro_sum[0], acc.gyro_n[0]),
            mean(acc.gyro_sum[1], acc.gyro_n[1]),
        ]
    } else {
        [Vector3::zeros(); IMU_COUNT]
    };

    let wire = [
        ImuCalWire {
            fine_rot: IDENTITY.fine_rot,
            gyro_bias: to_arr(gyro_bias_dps[0]),
        },
        ImuCalWire {
            fine_rot: row_major(&rotation),
            gyro_bias: to_arr(gyro_bias_dps[1]),
        },
    ];
    let stored = storage.store(&KEYS[0], &wire[0]).await && storage.store(&KEYS[1], &wire[1]).await;
    info!(
        "imu cal done: misalign {=f32} deg, {=usize} pairs, stored {=bool}",
        misalign_deg, acc.pairs, stored
    );

    ImuCalReport {
        pairs: acc.pairs,
        rotation,
        misalign_deg,
        residual_deg,
        coverage,
        gyro_bias_dps,
        accel_g: [
            mean_mag(acc.accel_mag_sum[0], acc.gravity_n[0]),
            mean_mag(acc.accel_mag_sum[1], acc.gravity_n[1]),
        ],
        gravity_n: acc.gravity_n,
        still_n: acc.gyro_n[0] + acc.gyro_n[1],
        peak_dps: acc.peak_dps,
        still_ok,
        stored,
    }
}

/// Kabsch fit (`a0 ~= R a1`); returns rotation, misalignment angle, residual, coverage.
fn solve_rotation(h: Matrix3<f32>, pairs: usize) -> (Matrix3<f32>, f32, f32, f32) {
    if pairs == 0 {
        return (Matrix3::identity(), 0.0, f32::NAN, 0.0);
    }
    let svd = h.svd(true, true);
    let u = svd.u.unwrap();
    let v_t = svd.v_t.unwrap();
    let mut d = Matrix3::identity();
    d[(2, 2)] = (u * v_t).determinant().signum();
    let r = v_t.transpose() * d * u.transpose();

    let misalign = Rotation3::from_matrix_unchecked(r).angle().to_degrees();
    // sum(a0 . R a1) = trace(R H); divide by count for the mean cosine.
    let mean_cos = ((r * h).trace() / pairs as f32).clamp(-1.0, 1.0);
    // Singular values reflect how the gravity vectors spread over the sphere:
    // smallest/largest near 1 means the poses covered all three axes.
    let sv = svd.singular_values;
    let coverage = if sv[0] > 0.0 { sv[2] / sv[0] } else { 0.0 };
    (r, misalign, libm::acosf(mean_cos).to_degrees(), coverage)
}

fn mean(sum: Vector3<f32>, n: usize) -> Vector3<f32> {
    if n == 0 {
        Vector3::zeros()
    } else {
        sum / n as f32
    }
}

fn mean_mag(sum: f32, n: usize) -> f32 {
    if n == 0 { f32::NAN } else { sum / n as f32 }
}

fn to_arr(v: Vector3<f32>) -> [f32; 3] {
    [v.x, v.y, v.z]
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
        writeln!(f, "imu cal -> {status}   {} gravity pairs\n", self.pairs)?;

        if self.pairs == 0 {
            writeln!(
                f,
                "  {:<21}\x1b[33mno gravity captured (hold stiller at each pose)\x1b[0m\n",
                "rotation"
            )?;
        } else {
            let r = &self.rotation;
            match Rotation3::from_matrix_unchecked(*r).axis() {
                Some(a) => writeln!(
                    f,
                    "  {:<21}{:.2} deg about [{:7.3}{:7.3}{:7.3} ]",
                    "rotation IMU1->IMU0", self.misalign_deg, a.x, a.y, a.z
                )?,
                None => writeln!(
                    f,
                    "  {:<21}{:.2} deg (near-aligned)",
                    "rotation IMU1->IMU0", self.misalign_deg
                )?,
            }
            writeln!(
                f,
                "  {:<21}residual {:.2} deg   coverage {:.2}",
                "", self.residual_deg, self.coverage
            )?;
            if self.coverage < GOOD_COVERAGE {
                writeln!(
                    f,
                    "  {:<21}\x1b[33m(!) low coverage - add poses tilted on edge/corner (gravity sideways)\x1b[0m",
                    ""
                )?;
            }
            if self.residual_deg > GOOD_RESIDUAL_DEG {
                writeln!(
                    f,
                    "  {:<21}\x1b[33m(!) high scatter - rest on a firm surface and hold stiller\x1b[0m",
                    ""
                )?;
            }
            write_row(f, "", r[(0, 0)], r[(0, 1)], r[(0, 2)])?;
            write_row(f, "", r[(1, 0)], r[(1, 1)], r[(1, 2)])?;
            write_row(f, "", r[(2, 0)], r[(2, 1)], r[(2, 2)])?;
            writeln!(f)?;
        }

        let (b0, b1) = (self.gyro_bias_dps[0], self.gyro_bias_dps[1]);
        if self.still_ok {
            bias_row(f, "gyro bias dps", "IMU0", b0, true)?;
            bias_row(f, "", "IMU1", b1, true)?;
            bias_row(f, "", "diff", b1 - b0, false)?;
            writeln!(f)?;
        } else {
            writeln!(
                f,
                "  {:<21}\x1b[31mboard moved (peak {:.1} dps); not stored\x1b[0m\n",
                "gyro bias", self.peak_dps
            )?;
        }

        let (a0, a1) = (self.accel_g[0], self.accel_g[1]);
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
            "samples", self.gravity_n[0], self.gravity_n[1], self.still_n, self.peak_dps
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
