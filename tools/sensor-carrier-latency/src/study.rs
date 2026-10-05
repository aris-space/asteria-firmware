//! Runs the fits on simulated logs of every scenario and compares them with
//! the latencies the simulator used.

use std::collections::BTreeMap;

use crate::log::Log;
use crate::models::fit_all;
use crate::sim::{SCENARIOS, TRUE_LATENCY_S, true_latency, write};

#[derive(Default)]
struct Tally {
    errors_ms: Vec<f64>,
    sigmas_ms: Vec<f64>,
    runs: usize,
}

pub fn run(seeds: u64, only: Option<&str>) -> Result<(), Box<dyn std::error::Error>> {
    let root = std::env::temp_dir().join("sensor-carrier-latency-study");
    println!("simulated logs in {}\n", root.display());
    for scenario in SCENARIOS
        .iter()
        .filter(|s| only.is_none_or(|name| s.name == name))
    {
        let mut tallies: BTreeMap<String, Tally> = BTreeMap::new();
        for seed in 0..seeds {
            let dir = root.join(format!("{}-{seed}", scenario.name));
            write(scenario, seed, &dir)?;
            let log = Log::read(&dir)?;
            for (_, report) in fit_all(&log) {
                for estimate in report.into_iter().flat_map(|r| r.estimates) {
                    let tally = tallies.entry(estimate.name.clone()).or_default();
                    tally.runs += 1;
                    if let Some(sigma) = estimate.sigma_s.filter(|&s| s < crate::OBSERVABLE_SIGMA_S)
                    {
                        tally
                            .errors_ms
                            .push((estimate.latency_s - true_latency(&estimate.name)) * 1e3);
                        tally.sigmas_ms.push(sigma * 1e3);
                    }
                }
            }
        }

        println!("{} — {}", scenario.name, scenario.description);
        println!("  sensor       true    found   error mean ± sd   reported σ");
        for (name, _) in TRUE_LATENCY_S {
            let Some(tally) = tallies.get(name) else {
                println!(
                    "  {name:<11} {:>5.1} ms     –   no data",
                    true_latency(name) * 1e3
                );
                continue;
            };
            let found = tally.errors_ms.len();
            if found == 0 {
                println!(
                    "  {name:<11} {:>5.1} ms   0/{}   not observable",
                    true_latency(name) * 1e3,
                    tally.runs
                );
                continue;
            }
            let (mean, sd) = mean_sd(&tally.errors_ms);
            let (sigma, _) = mean_sd(&tally.sigmas_ms);
            println!(
                "  {name:<11} {:>5.1} ms   {found}/{}   {mean:+7.2} ± {sd:5.2} ms   {sigma:6.2} ms",
                true_latency(name) * 1e3,
                tally.runs
            );
        }
        println!();
    }
    Ok(())
}

fn mean_sd(values: &[f64]) -> (f64, f64) {
    let n = values.len() as f64;
    let mean = values.iter().sum::<f64>() / n;
    let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / (n - 1.0).max(1.0);
    (mean, variance.sqrt())
}
