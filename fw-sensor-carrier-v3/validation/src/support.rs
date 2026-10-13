//! Reduce sampled readings for logging. Readings are printed, not range-checked:
//! a sensor passes as long as it responds and reads without error.

/// Samples taken per reported quantity; the value logged is their median.
pub const SAMPLES: usize = 20;

/// Median of `samples` (reorders in place); even counts average the two middle
/// values. Robust to the occasional interference spike, unlike a plain mean.
pub fn median(samples: &mut [f32]) -> f32 {
    samples.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let n = samples.len();
    if n % 2 == 1 {
        samples[n / 2]
    } else {
        (samples[n / 2 - 1] + samples[n / 2]) / 2.0
    }
}
