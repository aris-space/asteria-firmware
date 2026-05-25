//! Reduce sampled readings and range-check them. Bounds are wide on purpose:
//! they catch a dead or wrong-scale sensor, not calibration.

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

    /// Whether `value` is in range; logs an error if not. The value itself is
    /// logged by the caller.
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

// Bench-rest values (Earth field ~48 uT). Gyro bound is loose: at ±2000 dps the
// uncalibrated zero-rate bias is large, so it only catches a railed gyro.
pub const ACCEL_MAGNITUDE_G: Nominal = Nominal::new(0.7, 1.3);
pub const GYRO_MAGNITUDE_DPS: Nominal = Nominal::new(0.0, 100.0);
pub const IMU_TEMP_C: Nominal = Nominal::new(0.0, 50.0);
pub const BARO_PRESSURE_MBAR: Nominal = Nominal::new(700.0, 1100.0);
pub const BARO_TEMP_C: Nominal = Nominal::new(0.0, 50.0);
// Wide: raw |B| carries hard-iron offset, so this catches dead/railed, not the
// exact field.
pub const MAG_MAGNITUDE_NT: Nominal = Nominal::new(10_000.0, 130_000.0);
pub const SHT_TEMP_C: Nominal = Nominal::new(0.0, 50.0);
pub const SHT_RH_PCT: Nominal = Nominal::new(5.0, 95.0);

/// Number of samples averaged per sensor for a steadier estimate.
pub const SAMPLES: u32 = 16;

/// Magnetometer is reduced with a trimmed mean (drop lowest/highest `MAG_TRIM`)
/// to reject interference spikes.
pub const MAG_SAMPLES: usize = 20;
pub const MAG_TRIM: usize = 2;

/// Sort `samples`, drop the lowest and highest `trim`, and average the rest.
/// Reorders `samples` in place.
pub fn trimmed_mean(samples: &mut [f32], trim: usize) -> f32 {
    samples.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
    let kept = &samples[trim..samples.len() - trim];
    kept.iter().sum::<f32>() / kept.len() as f32
}
