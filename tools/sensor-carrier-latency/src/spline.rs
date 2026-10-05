//! Uniform cubic B-spline. A signal is `Σ c_j·B_j(t)`; at any time only four
//! basis functions are non-zero, which keeps the fit's matrices banded.

pub struct Spline {
    t0: f64,
    dt: f64,
    count: usize,
}

/// The four non-zero basis functions at one time, and their derivatives.
pub struct Basis {
    pub first: usize,
    pub value: [f64; 4],
    pub d1: [f64; 4],
    pub d2: [f64; 4],
}

impl Spline {
    /// Covers `[start, end]` with knots every `dt` seconds, plus `margin` on
    /// both sides so latency-shifted times stay inside.
    pub fn new(start: f64, end: f64, dt: f64, margin: f64) -> Self {
        let t0 = start - margin;
        let segments = ((end + margin - t0) / dt).ceil() as usize;
        Self {
            t0,
            dt,
            count: segments + 3,
        }
    }

    pub fn count(&self) -> usize {
        self.count
    }

    /// The basis at `t`, clamped into the covered range.
    pub fn basis(&self, t: f64) -> Basis {
        let u = ((t - self.t0) / self.dt).clamp(0.0, (self.count - 3) as f64 - 1e-9);
        let first = u.floor() as usize;
        let s = u - first as f64;
        let (s2, s3) = (s * s, s * s * s);
        let value = [
            (1.0 - s).powi(3) / 6.0,
            (3.0 * s3 - 6.0 * s2 + 4.0) / 6.0,
            (-3.0 * s3 + 3.0 * s2 + 3.0 * s + 1.0) / 6.0,
            s3 / 6.0,
        ];
        let d1 = [
            -(1.0 - s).powi(2) / 2.0,
            (3.0 * s2 - 4.0 * s) / 2.0,
            (-3.0 * s2 + 2.0 * s + 1.0) / 2.0,
            s2 / 2.0,
        ]
        .map(|v| v / self.dt);
        let d2 = [1.0 - s, 3.0 * s - 2.0, -3.0 * s + 1.0, s].map(|v| v / (self.dt * self.dt));
        Basis {
            first,
            value,
            d1,
            d2,
        }
    }
}
