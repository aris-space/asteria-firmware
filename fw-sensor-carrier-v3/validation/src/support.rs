//! Helpers for reducing sampled sensor readings and checking them against wide
//! nominal ranges. Bounds are intentionally loose: they catch a dead sensor or a
//! wrong scale, not a precise calibration, and tolerate orientation and handling.

use defmt::error;

/// Inclusive nominal range for an averaged reading.
pub struct Nominal {
    lo: f32,
    hi: f32,
}

impl Nominal {
    pub const fn new(lo: f32, hi: f32) -> Self {
        Self { lo, hi }
    }

    /// Return whether `value` sits inside the range, logging an error if not.
    /// The reading itself is logged by the caller on one combined line.
    pub fn check(&self, label: &str, quantity: &str, value: f32) -> bool {
        if value >= self.lo && value <= self.hi {
            true
        } else {
            error!(
                "{}: {} = {} OUT OF RANGE [{}, {}]",
                label, quantity, value, self.lo, self.hi
            );
            false
        }
    }
}

// Stationary on the bench: gravity vector magnitude ~1 g, room-ish temperature.
// Earth's field in Switzerland is ~48 uT (48000 nT). The gyro bound is loose: at
// ±2000 dps these uncalibrated parts show a large zero-rate bias, so this only
// catches a railed/runaway gyro, not a precise zero.
pub const ACCEL_MAGNITUDE_G: Nominal = Nominal::new(0.7, 1.3);
pub const GYRO_MAGNITUDE_DPS: Nominal = Nominal::new(0.0, 100.0);
pub const IMU_TEMP_C: Nominal = Nominal::new(0.0, 50.0);
pub const BARO_PRESSURE_MBAR: Nominal = Nominal::new(700.0, 1100.0);
pub const BARO_TEMP_C: Nominal = Nominal::new(0.0, 50.0);
// Wide because raw |B| includes per-location hard-iron offset (corrected in
// firmware); this catches a dead (~0) or railed sensor, not the exact field.
pub const MAG_MAGNITUDE_NT: Nominal = Nominal::new(10_000.0, 130_000.0);
pub const SHT_TEMP_C: Nominal = Nominal::new(0.0, 50.0);
pub const SHT_RH_PCT: Nominal = Nominal::new(5.0, 95.0);

/// Number of samples averaged per sensor for a steadier estimate.
pub const SAMPLES: u32 = 16;

/// The magnetometer is sampled over a longer window and reduced with a trimmed
/// mean (drop the lowest and highest `MAG_TRIM`) so an interference spike on a
/// single reading does not skew |B|.
pub const MAG_SAMPLES: usize = 20;
pub const MAG_TRIM: usize = 2;

/// Sort `samples`, drop the lowest and highest `trim`, and average the rest.
/// Reorders `samples` in place.
pub fn trimmed_mean(samples: &mut [f32], trim: usize) -> f32 {
    samples.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let kept = &samples[trim..samples.len() - trim];
    kept.iter().sum::<f32>() / kept.len() as f32
}
