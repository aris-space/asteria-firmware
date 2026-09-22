#![no_std]

//! Unit and epoch conversion at the SEF-light input boundary.

use asteria_sef_light::{
    GnssSample, ImuMeasurement, STANDARD_GRAVITY_MPS2, VerticalGnssMeasurement,
};

#[derive(Clone, Copy)]
pub struct BarometerReference {
    pressure_mbar: f32,
    scale_height_m: f32,
}

impl BarometerReference {
    pub fn new(pressure_mbar: f32, temperature_c: f32) -> Option<Self> {
        let temperature_k = temperature_c + 273.15;
        if !pressure_mbar.is_finite()
            || pressure_mbar <= 0.0
            || !temperature_k.is_finite()
            || temperature_k <= 0.0
        {
            return None;
        }
        // Dry-air gas constant divided by standard gravity, in m/K.
        Some(Self {
            pressure_mbar,
            scale_height_m: 29.271 * temperature_k,
        })
    }

    pub fn height_m(self, pressure_mbar: f32) -> Option<f32> {
        if !pressure_mbar.is_finite() || pressure_mbar <= 0.0 {
            return None;
        }
        let height_m = self.scale_height_m * libm::logf(self.pressure_mbar / pressure_mbar);
        height_m.is_finite().then_some(height_m)
    }
}

pub fn imu_measurement(acceleration_g: [f32; 3], angular_rate_deg_s: [f32; 3]) -> ImuMeasurement {
    const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
    ImuMeasurement {
        acceleration_body_mps2: acceleration_g.map(|value| value * STANDARD_GRAVITY_MPS2),
        angular_rate_body_rad_s: angular_rate_deg_s.map(|value| value * DEG_TO_RAD),
    }
}

pub struct GnssVerticalInput {
    pub launch_height_msl_m: f32,
    pub height_msl_m: f32,
    pub velocity_down_mps: f32,
    pub vertical_accuracy_mm: u32,
    pub speed_accuracy_mps: f32,
    pub fix_tier: u8,
    pub pdop_centi: u16,
}

pub fn gnss_measurement(input: GnssVerticalInput) -> GnssSample<VerticalGnssMeasurement> {
    let height_std_m = (input.vertical_accuracy_mm as f32 / 1000.0).max(0.5);
    let velocity_std_mps = if input.speed_accuracy_mps > 0.0 {
        input.speed_accuracy_mps.max(0.1)
    } else {
        1.0
    };
    GnssSample {
        measurement: VerticalGnssMeasurement {
            height_m: input.height_msl_m - input.launch_height_msl_m,
            velocity_mps: -input.velocity_down_mps,
            height_std_m,
            velocity_std_mps,
        },
        fix_tier: input.fix_tier,
        pdop_centi: input.pdop_centi,
    }
}

#[derive(Clone, Copy)]
pub struct GnssEpoch {
    pub receiver: usize,
    pub epoch_ms: u32,
    pub time_us: u64,
}

/// Only samples from distinct receivers and the same GNSS epoch may be blended.
pub fn pairable_epoch(first: GnssEpoch, second: GnssEpoch, maximum_span_us: u64) -> bool {
    first.receiver != second.receiver
        && first.epoch_ms == second.epoch_ms
        && first.time_us.abs_diff(second.time_us) <= maximum_span_us
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stationary_imu_and_gyroscope_units_match_sef_light() {
        let sample = imu_measurement([0.0, 0.0, -1.0], [180.0, 0.0, 0.0]);
        assert!((sample.acceleration_body_mps2[2] + STANDARD_GRAVITY_MPS2).abs() < 1e-5);
        assert!((sample.angular_rate_body_rad_s[0] - core::f32::consts::PI).abs() < 1e-5);
    }

    #[test]
    fn falling_pressure_produces_positive_launch_relative_height() {
        let reference = BarometerReference::new(900.0, 15.0).unwrap();
        assert!(reference.height_m(899.0).unwrap() > 9.0);
        assert_eq!(reference.height_m(900.0), Some(0.0));
        assert_eq!(reference.height_m(0.0), None);
    }

    #[test]
    fn gnss_uses_launch_reference_and_up_positive_velocity() {
        let sample = gnss_measurement(GnssVerticalInput {
            launch_height_msl_m: 1600.0,
            height_msl_m: 1602.5,
            velocity_down_mps: -3.0,
            vertical_accuracy_mm: 2500,
            speed_accuracy_mps: 0.0,
            fix_tier: 3,
            pdop_centi: 120,
        });
        assert_eq!(sample.measurement.height_m, 2.5);
        assert_eq!(sample.measurement.velocity_mps, 3.0);
        assert_eq!(sample.measurement.height_std_m, 2.5);
        assert_eq!(sample.measurement.velocity_std_mps, 1.0);
    }

    #[test]
    fn pairing_requires_a_shared_navigation_epoch() {
        let first = GnssEpoch {
            receiver: 0,
            epoch_ms: 10_000,
            time_us: 1_000_000,
        };
        assert!(pairable_epoch(
            first,
            GnssEpoch {
                receiver: 1,
                epoch_ms: 10_000,
                time_us: 1_120_000
            },
            150_000
        ));
        assert!(!pairable_epoch(
            first,
            GnssEpoch {
                receiver: 1,
                epoch_ms: 11_000,
                time_us: 1_020_000
            },
            150_000
        ));
        assert!(!pairable_epoch(
            first,
            GnssEpoch {
                receiver: 0,
                epoch_ms: 10_000,
                time_us: 1_020_000
            },
            150_000
        ));
        assert!(!pairable_epoch(
            first,
            GnssEpoch {
                receiver: 1,
                epoch_ms: 10_000,
                time_us: 1_200_000
            },
            150_000
        ));
    }
}
