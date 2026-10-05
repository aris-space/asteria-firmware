//! Fits fw-sensor-carrier-v3 calibration from an SD log session and prints
//! it as `cal set` lines to paste into the board's USB console.
//!
//! Gyro bias comes from still stretches, magnetometer hard and soft iron from
//! all orientations the board was turned through, both from raw counts.
//! Latencies come from modelling the motion as continuous-time B-splines:
//! IMU_1 against IMU_0's rotation, each magnetometer against IMU_0's gyro,
//! and the barometers and GNSS receivers against IMU_0's vertical
//! acceleration. They are relative to IMU_0. `mark still` and `mark tumble`
//! in the console narrow which stretches are used.
//!
//! `--plots` writes SVG plots to check the fits by eye.
//!
//! `simulate` writes logs with known latencies in the same format, and
//! `study` runs the fits on many simulated scenarios.

mod calibrate;
mod fit;
mod log;
mod models;
mod plot;
mod sim;
mod spline;
mod study;

use std::collections::BTreeMap;
use std::path::PathBuf;

use clap::{Parser, Subcommand};

use crate::calibrate::Calibration;
use crate::fit::{Estimate, FitReport};
use crate::log::{Log, STANDARD_GRAVITY, pressure_altitude_m};

/// A latency whose standard deviation exceeds this is not determined by the log.
const OBSERVABLE_SIGMA_S: f64 = 0.02;
const TIMELINE_WINDOW_S: f64 = 10.0;

#[derive(Parser)]
#[command(about = "Fit per-sensor latencies from sensor carrier SD logs")]
struct Args {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Fit the calibration of one logging session.
    Calibrate {
        /// A `LOGnnnn` directory, or the SD card root to use its newest session.
        log: PathBuf,
        /// Ignore samples before this many seconds into the session.
        #[arg(long, default_value_t = 0.0)]
        from: f64,
        /// Ignore samples after this many seconds into the session.
        #[arg(long, default_value_t = f64::INFINITY)]
        to: f64,
        /// Write SVG plots of the fits into this directory.
        #[arg(long)]
        plots: Option<PathBuf>,
    },
    /// Write a simulated session with known latencies.
    Simulate {
        /// One of the scenario names `study` lists.
        scenario: String,
        /// Directory to write the CSV files into.
        out: PathBuf,
        #[arg(long, default_value_t = 0)]
        seed: u64,
    },
    /// Fit simulated sessions of every scenario and compare with the truth.
    Study {
        /// Noise realizations per scenario.
        #[arg(long, default_value_t = 3)]
        seeds: u64,
        /// Run only this scenario.
        #[arg(long)]
        scenario: Option<String>,
        /// Save every simulated fit as CSV for plotting.
        #[arg(long)]
        csv: Option<PathBuf>,
    },
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    match Args::parse().command {
        Command::Calibrate {
            log,
            from,
            to,
            plots,
        } => {
            let mut log = Log::read(&log)?;
            print_overview(&log);
            log.crop(from, to);
            let estimates = fit_latencies(&log);
            let latencies = std::iter::once(("IMU_0".to_string(), 0.0))
                .chain(
                    estimates
                        .iter()
                        .filter(|e| e.sigma_s.is_some_and(|sigma| sigma < OBSERVABLE_SIGMA_S))
                        .map(|e| (e.name.clone(), e.latency_s)),
                )
                .collect();
            let calibration = Calibration::fit(&log);
            calibration.print(&log, &latencies);
            if let Some(dir) = plots {
                plot::write(&dir, &log, &calibration, &estimates)?;
                println!("\nwrote plots to {}", dir.display());
            }
        }
        Command::Simulate {
            scenario,
            out,
            seed,
        } => {
            let scenario = sim::SCENARIOS
                .iter()
                .find(|s| s.name == scenario)
                .ok_or("unknown scenario; `study` lists them")?;
            sim::write(scenario, seed, &out)?;
            println!(
                "wrote {} ({}) to {}",
                scenario.name,
                scenario.description,
                out.display()
            );
        }
        Command::Study {
            seeds,
            scenario,
            csv,
        } => study::run(seeds, scenario.as_deref(), csv.as_deref())?,
    }
    Ok(())
}

fn print_overview(log: &Log) {
    let duration = log.duration_s();
    let rate = |count: usize| count as f64 / duration.max(1e-9);
    println!("{} — {:.0} s", log.dir.display(), duration);
    for (name, count) in [
        ("IMU_0", log.imu[0].len()),
        ("IMU_1", log.imu[1].len()),
        ("MAG_BUS_1", log.mag[0].len()),
        ("MAG_BUS_2", log.mag[1].len()),
        ("BARO_BUS_1", log.baro[0].len()),
        ("BARO_BUS_2", log.baro[1].len()),
    ] {
        println!("  {name:<11} {count:>8} samples  {:7.1} Hz", rate(count));
    }
    for (i, samples) in log.gnss.iter().enumerate() {
        let fixes = samples
            .iter()
            .filter(|s| s.fix_ok && matches!(s.fix_type, 3 | 4))
            .count();
        println!(
            "  GNSS_{i}      {:>8} epochs  {:7.1} Hz, {:.0} % with a 3D fix",
            samples.len(),
            rate(samples.len()),
            100.0 * fixes as f64 / samples.len().max(1) as f64
        );
    }
    if log.dropped > 0 {
        println!(
            "  SD writer dropped {} readings (see DROPS.CSV)",
            log.dropped
        );
    }
    for (file, rows) in &log.malformed {
        println!("  skipped {rows} malformed rows in {file}");
    }

    // Motion per window, to choose --from/--to: rotation for IMU_1 and the
    // magnetometers, vertical motion for the barometers and GNSS.
    println!("\n  window      rotation   accel spread   baro range   GNSS fix");
    let windows = (duration / TIMELINE_WINDOW_S).ceil() as usize;
    for w in 0..windows {
        let (start, end) = (
            w as f64 * TIMELINE_WINDOW_S,
            (w + 1) as f64 * TIMELINE_WINDOW_S,
        );
        let inside = |t: f64| (start..end).contains(&t);
        let imu: Vec<_> = log.imu[0].iter().filter(|s| inside(s.t)).collect();
        let rotation_dps = rms(imu
            .iter()
            .map(|s| s.gyro_dps.iter().map(|v| v * v).sum::<f64>().sqrt()));
        let accel = imu
            .iter()
            .map(|s| s.accel_g.iter().map(|v| v * v).sum::<f64>().sqrt() * STANDARD_GRAVITY);
        let accel_spread = spread(accel);
        let altitudes: Vec<f64> = log.baro[0]
            .iter()
            .filter(|s| inside(s.t))
            .map(|s| pressure_altitude_m(s.pressure_mbar))
            .collect();
        let baro_range = altitudes.iter().copied().fold(f64::NEG_INFINITY, f64::max)
            - altitudes.iter().copied().fold(f64::INFINITY, f64::min);
        let epochs: Vec<_> = log.gnss.iter().flatten().filter(|s| inside(s.t)).collect();
        let fixes = epochs
            .iter()
            .filter(|s| s.fix_ok && matches!(s.fix_type, 3 | 4))
            .count();
        println!(
            "  {:>4}–{:<4} s  {:6.1} dps   {:6.2} m/s²   {:7.2} m   {:5.0} %",
            start,
            end,
            rotation_dps,
            accel_spread,
            if altitudes.is_empty() {
                0.0
            } else {
                baro_range
            },
            100.0 * fixes as f64 / epochs.len().max(1) as f64
        );
    }
    println!();
}

/// Prints every latency fit and returns its estimates.
fn fit_latencies(log: &Log) -> Vec<Estimate> {
    let mut estimates = Vec::new();
    for (title, report) in models::fit_all(log) {
        println!("{title}");
        match report {
            Some(report) => {
                print_report(&report);
                estimates.extend(report.estimates);
            }
            None => println!("  not enough data\n"),
        }
    }
    if let Some(offset_s) = gnss_epoch_offset(log) {
        println!(
            "GNSS_1 − GNSS_0 by matching iTOW: {:+.1} ms (cross-check)\n",
            offset_s * 1e3
        );
    }
    estimates
}

fn print_report(report: &FitReport) {
    for estimate in &report.estimates {
        match estimate.sigma_s {
            Some(sigma) if sigma < OBSERVABLE_SIGMA_S => println!(
                "  {:<11} {:+8.2} ms ± {:.2}",
                estimate.name,
                estimate.latency_s * 1e3,
                sigma * 1e3
            ),
            _ => println!("  {:<11} not observable (too little motion)", estimate.name),
        }
    }
    println!(
        "  {} rows, normalized residual RMS {:.2}\n",
        report.rows, report.normalized_rms
    );
}

/// The median receipt-time difference of epochs both receivers reported.
fn gnss_epoch_offset(log: &Log) -> Option<f64> {
    let first: BTreeMap<u32, f64> = log.gnss[0].iter().map(|s| (s.itow_ms, s.t)).collect();
    let mut offsets: Vec<f64> = log.gnss[1]
        .iter()
        .filter_map(|s| Some(s.t - first.get(&s.itow_ms)?))
        .collect();
    offsets.sort_by(f64::total_cmp);
    offsets.get(offsets.len() / 2).copied()
}

fn rms(values: impl Iterator<Item = f64>) -> f64 {
    let (sum, n) = values.fold((0.0, 0usize), |(sum, n), v| (sum + v * v, n + 1));
    (sum / n.max(1) as f64).sqrt()
}

fn spread(values: impl Iterator<Item = f64>) -> f64 {
    let values: Vec<f64> = values.collect();
    let n = values.len().max(1) as f64;
    let mean = values.iter().sum::<f64>() / n;
    (values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n).sqrt()
}
