//! Fits each sensor's latency from one fw-sensor-carrier-v3 SD log directory.
//!
//! The motion is modelled as continuous-time B-splines. IMU_1 is fitted
//! against IMU_0's rotation, each magnetometer against IMU_0's gyro, and the
//! barometers and GNSS receivers against IMU_0's vertical acceleration.
//! Latencies are relative to IMU_0 and printed as `cal latency` commands.

mod fit;
mod log;
mod models;
mod spline;

use std::collections::HashMap;
use std::path::PathBuf;

use clap::Parser;

use crate::fit::{FitReport, Model, fit};
use crate::log::Log;

/// A latency whose standard deviation exceeds this is not determined by the log.
const OBSERVABLE_SIGMA_S: f64 = 0.02;

#[derive(Parser)]
#[command(about = "Fit per-sensor latencies from a sensor carrier SD log")]
struct Args {
    /// A `LOGnnnn` directory from the SD card.
    log: PathBuf,
    /// Ignore samples before this many seconds into the log.
    #[arg(long, default_value_t = 0.0)]
    from: f64,
    /// Ignore samples after this many seconds into the log.
    #[arg(long, default_value_t = f64::INFINITY)]
    to: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let mut log = Log::read(&args.log)?;
    log.crop(args.from, args.to);

    let models: Vec<(&str, Option<Box<dyn Model>>)> = vec![
        (
            "IMU_1 vs IMU_0 rotation",
            models::ImuTwin::new(&log).map(boxed),
        ),
        (
            "MAG_BUS_1 vs gyro",
            models::MagRotation::new(&log, 0).map(boxed),
        ),
        (
            "MAG_BUS_2 vs gyro",
            models::MagRotation::new(&log, 1).map(boxed),
        ),
        (
            "Baro and GNSS vs vertical acceleration",
            models::Vertical::new(&log).map(boxed),
        ),
    ];

    let mut latencies = HashMap::from([("IMU_0".to_string(), 0.0)]);
    for (title, model) in models {
        println!("{title}");
        let Some(model) = model else {
            println!("  not enough data\n");
            continue;
        };
        let report = fit(model.as_ref());
        print_report(&report);
        for estimate in report.estimates {
            if estimate
                .sigma_s
                .is_some_and(|sigma| sigma < OBSERVABLE_SIGMA_S)
            {
                latencies.insert(estimate.name, estimate.latency_s);
            }
        }
    }

    if let Some(offset_s) = gnss_epoch_offset(&log) {
        println!(
            "GNSS_1 − GNSS_0 by matching iTOW: {:+.1} ms (cross-check)\n",
            offset_s * 1e3
        );
    }

    println!("cal commands (latencies relative to IMU_0):");
    let mut names: Vec<_> = latencies.keys().cloned().collect();
    names.sort();
    for name in names {
        println!(
            "cal latency {name} {}",
            (latencies[&name] * 1e6).round() as i64
        );
    }
    Ok(())
}

fn boxed<M: Model + 'static>(model: M) -> Box<dyn Model> {
    Box::new(model)
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
            _ => println!(
                "  {:<11} not observable (too little motion in this log)",
                estimate.name
            ),
        }
    }
    println!(
        "  {} rows, normalized residual RMS {:.2}\n",
        report.rows, report.normalized_rms
    );
}

/// The median receipt-time difference of epochs both receivers reported.
fn gnss_epoch_offset(log: &Log) -> Option<f64> {
    let first: HashMap<u32, f64> = log.gnss[0].iter().map(|s| (s.itow_ms, s.t)).collect();
    let mut offsets: Vec<f64> = log.gnss[1]
        .iter()
        .filter_map(|s| Some(s.t - first.get(&s.itow_ms)?))
        .collect();
    offsets.sort_by(f64::total_cmp);
    offsets.get(offsets.len() / 2).copied()
}
