//! Reads the CSV files of one `LOGnnnn` directory. Times are seconds since
//! the first IMU sample, taken from `raw_us` so a fit does not depend on the
//! latencies stored on the board.

use std::path::{Path, PathBuf};

use serde::Deserialize;

pub const STANDARD_GRAVITY: f64 = 9.806_65;

pub struct Log {
    pub dir: PathBuf,
    pub imu: [Vec<Imu>; 2],
    pub mag: [Vec<Mag>; 2],
    pub baro: [Vec<Baro>; 2],
    pub gnss: [Vec<Gnss>; 2],
    /// IMU_0 chain attitude, body to NED, as `(t, [w, x, y, z])`.
    pub attitude: Vec<(f64, [f64; 4])>,
    /// Readings the SD writer dropped, summed over the session.
    pub dropped: u64,
    /// Rows that could not be parsed, e.g. a line cut off by power loss.
    pub malformed: Vec<(&'static str, usize)>,
}

#[derive(Clone, Copy)]
pub struct Imu {
    pub t: f64,
    pub accel_g: [f64; 3],
    pub gyro_dps: [f64; 3],
}

#[derive(Clone, Copy)]
pub struct Mag {
    pub t: f64,
    pub field_nt: [f64; 3],
}

#[derive(Clone, Copy)]
pub struct Baro {
    pub t: f64,
    pub pressure_mbar: f64,
}

#[derive(Clone, Copy)]
pub struct Gnss {
    pub t: f64,
    pub itow_ms: u32,
    pub fix_type: u8,
    pub fix_ok: bool,
    pub height_msl_m: f64,
    pub velocity_down_mps: f64,
    pub vertical_accuracy_mm: f64,
    pub speed_accuracy_mps: f64,
}

#[derive(Deserialize)]
struct ImuRow {
    raw_us: u64,
    imu: usize,
    ax_g: f64,
    ay_g: f64,
    az_g: f64,
    gx_dps: f64,
    gy_dps: f64,
    gz_dps: f64,
}

#[derive(Deserialize)]
struct MagRow {
    raw_us: u64,
    mag: usize,
    x_nt: f64,
    y_nt: f64,
    z_nt: f64,
}

#[derive(Deserialize)]
struct BaroRow {
    raw_us: u64,
    baro: usize,
    pressure_mbar: f64,
}

#[derive(Deserialize)]
struct GnssRow {
    raw_us: u64,
    gnss: usize,
    itow_ms: u32,
    fix_type: u8,
    fix_ok: u8,
    height_msl_m: f64,
    velocity_down_mps: f64,
    vertical_accuracy_mm: f64,
    speed_accuracy_mps: f64,
}

#[derive(Deserialize)]
struct DropRow {
    state: u64,
    imu: u64,
    mag: u64,
    gnss: u64,
    baro: u64,
    dht: u64,
}

#[derive(Deserialize)]
struct StateRow {
    cal_us: u64,
    imu: usize,
    qw: f64,
    qx: f64,
    qy: f64,
    qz: f64,
}

impl Log {
    /// Reads `path`, which is either a `LOGnnnn` directory or a directory
    /// holding them, such as the SD card root; then the newest one is used.
    pub fn read(path: &Path) -> Result<Self, Box<dyn std::error::Error>> {
        let dir = session_dir(path)?;
        let mut malformed = Vec::new();
        let imu_rows: Vec<ImuRow> = read_csv(&dir, "IMU.CSV", &mut malformed)?;
        let mag_rows: Vec<MagRow> = read_csv(&dir, "MAG.CSV", &mut malformed)?;
        let baro_rows: Vec<BaroRow> = read_csv(&dir, "BARO.CSV", &mut malformed)?;
        let gnss_rows: Vec<GnssRow> = read_csv(&dir, "GNSS.CSV", &mut malformed)?;
        let state_rows: Vec<StateRow> = read_csv(&dir, "STATE.CSV", &mut malformed)?;
        let drop_rows: Vec<DropRow> = read_csv(&dir, "DROPS.CSV", &mut malformed)?;

        let start_us = imu_rows
            .iter()
            .map(|row| row.raw_us)
            .min()
            .ok_or("IMU.CSV has no rows")?;
        let t = |us: u64| (us as f64 - start_us as f64) * 1e-6;

        let mut log = Log {
            dir,
            imu: [Vec::new(), Vec::new()],
            mag: [Vec::new(), Vec::new()],
            baro: [Vec::new(), Vec::new()],
            gnss: [Vec::new(), Vec::new()],
            attitude: Vec::new(),
            dropped: drop_rows
                .iter()
                .map(|d| d.state + d.imu + d.mag + d.gnss + d.baro + d.dht)
                .sum(),
            malformed,
        };
        for row in imu_rows.into_iter().filter(|row| row.imu < 2) {
            log.imu[row.imu].push(Imu {
                t: t(row.raw_us),
                accel_g: [row.ax_g, row.ay_g, row.az_g],
                gyro_dps: [row.gx_dps, row.gy_dps, row.gz_dps],
            });
        }
        for row in mag_rows.into_iter().filter(|row| row.mag < 2) {
            log.mag[row.mag].push(Mag {
                t: t(row.raw_us),
                field_nt: [row.x_nt, row.y_nt, row.z_nt],
            });
        }
        for row in baro_rows.into_iter().filter(|row| row.baro < 2) {
            log.baro[row.baro].push(Baro {
                t: t(row.raw_us),
                pressure_mbar: row.pressure_mbar,
            });
        }
        for row in gnss_rows.into_iter().filter(|row| row.gnss < 2) {
            log.gnss[row.gnss].push(Gnss {
                t: t(row.raw_us),
                itow_ms: row.itow_ms,
                fix_type: row.fix_type,
                fix_ok: row.fix_ok != 0,
                height_msl_m: row.height_msl_m,
                velocity_down_mps: row.velocity_down_mps,
                vertical_accuracy_mm: row.vertical_accuracy_mm,
                speed_accuracy_mps: row.speed_accuracy_mps,
            });
        }
        for row in state_rows.into_iter().filter(|row| row.imu == 0) {
            log.attitude
                .push((t(row.cal_us), [row.qw, row.qx, row.qy, row.qz]));
        }
        for samples in &mut log.imu {
            samples.sort_by(|a, b| a.t.total_cmp(&b.t));
        }
        for samples in &mut log.mag {
            samples.sort_by(|a, b| a.t.total_cmp(&b.t));
        }
        for samples in &mut log.baro {
            samples.sort_by(|a, b| a.t.total_cmp(&b.t));
        }
        for samples in &mut log.gnss {
            samples.sort_by(|a, b| a.t.total_cmp(&b.t));
        }
        log.attitude.sort_by(|a, b| a.0.total_cmp(&b.0));
        Ok(log)
    }

    pub fn duration_s(&self) -> f64 {
        self.imu[0].last().map_or(0.0, |s| s.t)
    }

    /// Keeps only samples within `[from, to]` seconds.
    pub fn crop(&mut self, from: f64, to: f64) {
        let keep = |t: f64| (from..=to).contains(&t);
        for samples in &mut self.imu {
            samples.retain(|s| keep(s.t));
        }
        for samples in &mut self.mag {
            samples.retain(|s| keep(s.t));
        }
        for samples in &mut self.baro {
            samples.retain(|s| keep(s.t));
        }
        for samples in &mut self.gnss {
            samples.retain(|s| keep(s.t));
        }
    }

    /// IMU_0 attitude at `t`, interpolated between estimator outputs.
    pub fn attitude_at(&self, t: f64) -> Option<[f64; 4]> {
        let i = self.attitude.partition_point(|&(ts, _)| ts < t);
        let (t1, q1) = *self.attitude.get(i)?;
        let (t0, mut q0) = *self.attitude.get(i.checked_sub(1)?)?;
        if q0.iter().zip(&q1).map(|(a, b)| a * b).sum::<f64>() < 0.0 {
            q0 = q0.map(|v| -v);
        }
        let u = (t - t0) / (t1 - t0);
        let q: [f64; 4] = std::array::from_fn(|k| q0[k] + u * (q1[k] - q0[k]));
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        Some(q.map(|v| v / norm))
    }
}

/// ISA pressure altitude, the same conversion the firmware uses.
pub fn pressure_altitude_m(pressure_mbar: f64) -> f64 {
    44_330.0 * (1.0 - (pressure_mbar / 1_013.25).powf(0.190_294_95))
}

/// Averages consecutive groups of `n` IMU samples, timestamped at their mean.
pub fn decimate(samples: &[Imu], n: usize) -> Vec<Imu> {
    samples
        .chunks_exact(n)
        .map(|chunk| {
            let mean = |f: &dyn Fn(&Imu) -> f64| chunk.iter().map(f).sum::<f64>() / n as f64;
            Imu {
                t: mean(&|s| s.t),
                accel_g: std::array::from_fn(|k| mean(&|s| s.accel_g[k])),
                gyro_dps: std::array::from_fn(|k| mean(&|s| s.gyro_dps[k])),
            }
        })
        .collect()
}

/// Robust noise standard deviation of a slowly varying signal, from the
/// median absolute difference between successive samples.
pub fn noise_sigma(values: impl Iterator<Item = f64>, floor: f64) -> f64 {
    let values: Vec<f64> = values.collect();
    let mut diffs: Vec<f64> = values.windows(2).map(|w| (w[1] - w[0]).abs()).collect();
    if diffs.is_empty() {
        return floor;
    }
    diffs.sort_by(f64::total_cmp);
    (diffs[diffs.len() / 2] / (0.6745 * std::f64::consts::SQRT_2)).max(floor)
}

/// The `LOGnnnn` directory at `path`, or the newest one inside it.
fn session_dir(path: &Path) -> Result<PathBuf, Box<dyn std::error::Error>> {
    if path.join("IMU.CSV").exists() {
        return Ok(path.to_path_buf());
    }
    let newest = std::fs::read_dir(path)?
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|dir| {
            dir.file_name()
                .and_then(|name| name.to_str())
                .and_then(|name| name.strip_prefix("LOG"))
                .is_some_and(|number| number.len() == 4 && number.parse::<u16>().is_ok())
                && dir.join("IMU.CSV").exists()
        })
        .max();
    newest.ok_or_else(|| format!("no LOGnnnn directory with IMU.CSV in {}", path.display()).into())
}

/// The rows of a CSV file. Rows that do not parse are skipped and counted in
/// `malformed`; a missing file has no rows.
fn read_csv<T: for<'de> Deserialize<'de>>(
    dir: &Path,
    name: &'static str,
    malformed: &mut Vec<(&'static str, usize)>,
) -> Result<Vec<T>, Box<dyn std::error::Error>> {
    let path = dir.join(name);
    if !path.exists() {
        return Ok(Vec::new());
    }
    let mut reader = csv::ReaderBuilder::new().flexible(true).from_path(path)?;
    let mut rows = Vec::new();
    let mut skipped = 0;
    for row in reader.deserialize() {
        match row {
            Ok(row) => rows.push(row),
            Err(_) => skipped += 1,
        }
    }
    if skipped > 0 {
        malformed.push((name, skipped));
    }
    Ok(rows)
}
