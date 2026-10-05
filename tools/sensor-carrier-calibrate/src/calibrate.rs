//! Fits each board's calibration from raw sensor counts and prints it as
//! `cal set` console lines. Units and axes match the firmware's
//! `calibration/<kind>.rs`.

use std::collections::BTreeMap;

use magcal::Solver;

use crate::log::{Imu, Log};

const GYRO_DPS_PER_LSB: f64 = 0.07;
const MAG_NT_PER_LSB: f64 = 150.0;
const IMU_HZ: f64 = 833.0;
// A one-second window counts as still if no axis varies more than this.
const STILL_WINDOW_S: f64 = 1.0;
const STILL_MAX_SPREAD_DPS: f64 = 0.5;
const MIN_STILL_S: f64 = 5.0;
// Plausible Earth-field magnitude, as in the firmware.
const FIELD_RANGE_NT: std::ops::RangeInclusive<f64> = 22_000.0..=67_000.0;

/// LSM6DSO32 sensor-to-board remap: negate x and z.
fn imu_board_dps(raw: [f64; 3]) -> [f64; 3] {
    [-raw[0], raw[1], -raw[2]].map(|counts| counts * GYRO_DPS_PER_LSB)
}

/// LSM303AGR sensor-to-board remap: negate all axes.
fn mag_board_counts(raw: [i16; 3]) -> [i16; 3] {
    raw.map(i16::saturating_neg)
}

/// Prints one `cal set` line per sensor whose calibration the log determines,
/// and a comment for every sensor it does not.
pub fn print(log: &Log, latencies: &BTreeMap<String, f64>) {
    let name = log
        .dir
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("offline");
    let latency = |sensor: &str| latencies.get(sensor).map(|s| (s * 1e6).round() as i64);
    let still = intervals(log, "still");
    let tumble = intervals(log, "tumble");

    println!("Paste into the console, then `reset` and `cal show`:");
    for imu in 0..2 {
        let sensor = format!("IMU_{imu}");
        let still_samples = if still.is_empty() {
            auto_still(&log.imu[imu])
        } else {
            within(&log.imu[imu], &still, |s| s.t)
        };
        let still_s = still_samples.len() as f64 / IMU_HZ;
        match (gyro_bias(&still_samples), latency(&sensor)) {
            (Some(bias), Some(latency_us)) => println!(
                "cal set {sensor} name={name} latency_us={latency_us} gyro_bias_dps={:.4},{:.4},{:.4}",
                bias[0], bias[1], bias[2]
            ),
            (None, _) => {
                println!("# {sensor}: only {still_s:.1} s still, need {MIN_STILL_S:.0} s; no line")
            }
            (_, None) => println!("# {sensor}: latency not observable; no line"),
        }
    }
    for mag in 0..2 {
        let sensor = format!("MAG_BUS_{}", mag + 1);
        let samples = if tumble.is_empty() {
            log.mag[mag].iter().collect()
        } else {
            within(&log.mag[mag], &tumble, |s| s.t)
        };
        let mut solver = Solver::new();
        for sample in &samples {
            solver.push_sample(mag_board_counts(sample.field_raw));
        }
        let fit = solver
            .solve()
            .ok()
            .filter(|fit| FIELD_RANGE_NT.contains(&(fit.field_strength as f64 * MAG_NT_PER_LSB)));
        match (fit, latency(&sensor)) {
            (Some(fit), Some(latency_us)) => {
                let h = fit.hard_iron.map(|counts| counts as f64 * MAG_NT_PER_LSB);
                let s = fit.soft_iron;
                println!(
                    "# {sensor}: {:?} fit, field {:.1} uT, error {:.2} %, {} samples",
                    fit.tier,
                    fit.field_strength as f64 * MAG_NT_PER_LSB / 1000.0,
                    fit.fit_error_percent,
                    samples.len()
                );
                println!(
                    "cal set {sensor} name={name} latency_us={latency_us} hard_nt={:.1},{:.1},{:.1} soft={:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5},{:.5}",
                    h[0],
                    h[1],
                    h[2],
                    s[0][0],
                    s[0][1],
                    s[0][2],
                    s[1][0],
                    s[1][1],
                    s[1][2],
                    s[2][0],
                    s[2][1],
                    s[2][2],
                );
            }
            (None, _) => println!(
                "# {sensor}: no plausible fit from {} samples (tumble through all orientations); no line",
                samples.len()
            ),
            (_, None) => println!("# {sensor}: latency not observable; no line"),
        }
    }
    for sensor in ["BARO_BUS_1", "BARO_BUS_2", "GNSS_0", "GNSS_1"] {
        match latency(sensor) {
            Some(latency_us) => println!("cal set {sensor} name={name} latency_us={latency_us}"),
            None => println!("# {sensor}: latency not observable; no line"),
        }
    }
}

/// Time ranges that start at a `label` mark and end at the next mark.
fn intervals(log: &Log, label: &str) -> Vec<(f64, f64)> {
    log.marks
        .iter()
        .enumerate()
        .filter(|(_, (_, l))| l == label)
        .map(|(i, &(start, _))| {
            let end = log.marks.get(i + 1).map_or(f64::INFINITY, |&(t, _)| t);
            (start, end)
        })
        .collect()
}

fn within<'a, T>(samples: &'a [T], ranges: &[(f64, f64)], t: impl Fn(&T) -> f64) -> Vec<&'a T> {
    samples
        .iter()
        .filter(|s| {
            ranges
                .iter()
                .any(|&(start, end)| (start..end).contains(&t(s)))
        })
        .collect()
}

/// Samples in one-second windows where the board did not rotate.
fn auto_still(samples: &[Imu]) -> Vec<&Imu> {
    samples
        .chunk_by(|a, b| (a.t / STILL_WINDOW_S).floor() == (b.t / STILL_WINDOW_S).floor())
        .filter(|window| {
            (0..3).all(|axis| {
                let rates: Vec<f64> = window
                    .iter()
                    .map(|s| imu_board_dps(s.gyro_raw)[axis])
                    .collect();
                let mean = rates.iter().sum::<f64>() / rates.len() as f64;
                let variance =
                    rates.iter().map(|r| (r - mean).powi(2)).sum::<f64>() / rates.len() as f64;
                variance.sqrt() < STILL_MAX_SPREAD_DPS
            })
        })
        .flatten()
        .collect()
}

/// Mean board-frame rate while still, which is the gyro bias.
fn gyro_bias(still: &[&Imu]) -> Option<[f64; 3]> {
    (still.len() as f64 / IMU_HZ >= MIN_STILL_S).then(|| {
        std::array::from_fn(|axis| {
            still
                .iter()
                .map(|s| imu_board_dps(s.gyro_raw)[axis])
                .sum::<f64>()
                / still.len() as f64
        })
    })
}
