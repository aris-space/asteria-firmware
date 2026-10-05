//! Separable least squares by variable projection. Every measurement row is
//! linear in the model's unknowns (spline coefficients, biases) and depends
//! non-linearly only on the latencies. For given latencies the unknowns are
//! solved exactly; Levenberg-Marquardt then fits the latencies on the
//! remaining residuals, using Kaufman's approximation of their Jacobian.

use levenberg_marquardt::{LeastSquaresProblem, LevenbergMarquardt};
use nalgebra::storage::Owned;
use nalgebra::{DMatrix, DVector, Dyn};
use nalgebra_sparse::factorization::CscCholesky;
use nalgebra_sparse::{CooMatrix, CscMatrix};

/// One measurement: `Σ entries·x = rhs ± sigma`.
pub struct Row {
    pub entries: Vec<(usize, f64)>,
    pub rhs: f64,
    pub sigma: f64,
    /// The derivative of `entries` with respect to one latency, if any.
    pub d_latency: Option<(usize, Vec<(usize, f64)>)>,
}

pub trait Model {
    fn unknowns(&self) -> usize;
    fn latency_names(&self) -> Vec<String>;
    /// All rows for the given latencies, always the same rows in the same order.
    fn rows(&self, latencies: &[f64]) -> Vec<Row>;
}

pub struct Estimate {
    pub name: String,
    pub latency_s: f64,
    /// `None` if the data cannot determine this latency.
    pub sigma_s: Option<f64>,
    /// The fit at latencies around the estimate.
    pub profile: Vec<ProfilePoint>,
}

pub struct ProfilePoint {
    pub latency_s: f64,
    /// Increase of the normalized χ² over the best fit.
    pub increase: f64,
    /// The increase the covariance predicts.
    pub predicted: f64,
}

// The covariance describes the fit only near its minimum. Without real
// motion that minimum is a ripple in the noise, and moving the latency changes
// the fit far less than the covariance predicts. A latency counts as determined
// only if the fit at least TEST_MIN_STEPS profile steps away worsens by at least
// a quarter of the predicted amount, and clearly (five standard deviations).
const PROFILE_STEP_S: f64 = 0.005;
// The profile spans ±PROFILE_STEPS steps around the estimate.
const PROFILE_STEPS: i32 = 20;
const TEST_MIN_STEPS: i32 = 10;
const MIN_PREDICTED_FRACTION: f64 = 0.25;
const MIN_CHI2_INCREASE: f64 = 25.0;

pub struct FitReport {
    pub estimates: Vec<Estimate>,
    pub rows: usize,
    /// Root mean square of the whitened residuals; near 1 if the noise model fits.
    pub normalized_rms: f64,
}

pub fn fit(model: &dyn Model) -> FitReport {
    let names = model.latency_names();
    let mut problem = Projection {
        model,
        latencies: DVector::zeros(names.len()),
        residuals: DVector::zeros(0),
        jacobian: DMatrix::zeros(0, 0),
    };
    problem.update();
    let (problem, _) = LevenbergMarquardt::new().minimize(problem);

    let rows = problem.residuals.len();
    let dof = rows.saturating_sub(model.unknowns() + names.len()).max(1);
    let chi2 = problem.residuals.norm_squared();
    let covariance = (problem.jacobian.transpose() * &problem.jacobian)
        .try_inverse()
        .map(|inverse| inverse * (chi2 / dof as f64));
    let scale = chi2 / dof as f64;
    let best = problem.latencies.clone();
    let estimates = names
        .into_iter()
        .enumerate()
        .map(|(i, name)| {
            let sigma = covariance
                .as_ref()
                .map(|c| c[(i, i)].sqrt())
                .filter(|sigma| sigma.is_finite());
            let profile: Vec<ProfilePoint> = (-PROFILE_STEPS..=PROFILE_STEPS)
                .map(|step| {
                    let shift = f64::from(step) * PROFILE_STEP_S;
                    let mut latencies = best.clone();
                    latencies[i] += shift;
                    let mut shifted = Projection {
                        latencies,
                        ..problem.empty()
                    };
                    shifted.update();
                    ProfilePoint {
                        latency_s: best[i] + shift,
                        increase: (shifted.residuals.norm_squared() - chi2) / scale,
                        predicted: sigma.map_or(0.0, |sigma| (shift / sigma).powi(2)),
                    }
                })
                .collect();
            let determined = sigma.is_some()
                && profile.iter().zip(-PROFILE_STEPS..).all(|(point, step)| {
                    step.abs() < TEST_MIN_STEPS
                        || point.increase
                            > MIN_CHI2_INCREASE.max(MIN_PREDICTED_FRACTION * point.predicted)
                });
            Estimate {
                name,
                latency_s: best[i],
                sigma_s: sigma.filter(|_| determined),
                profile,
            }
        })
        .collect();
    FitReport {
        estimates,
        rows,
        normalized_rms: (chi2 / rows.max(1) as f64).sqrt(),
    }
}

struct Projection<'a> {
    model: &'a dyn Model,
    latencies: DVector<f64>,
    residuals: DVector<f64>,
    jacobian: DMatrix<f64>,
}

impl<'a> Projection<'a> {
    fn empty(&self) -> Projection<'a> {
        Projection {
            model: self.model,
            latencies: self.latencies.clone(),
            residuals: DVector::zeros(0),
            jacobian: DMatrix::zeros(0, 0),
        }
    }

    fn update(&mut self) {
        let rows = self.model.rows(self.latencies.as_slice());
        let n = self.model.unknowns();
        let weight = |row: &Row| 1.0 / row.sigma;

        let mut normal = CooMatrix::new(n, n);
        let mut rhs = DVector::zeros(n);
        for row in &rows {
            let w2 = weight(row) * weight(row);
            for &(i, a) in &row.entries {
                rhs[i] += w2 * a * row.rhs;
                for &(j, b) in &row.entries {
                    normal.push(i, j, w2 * a * b);
                }
            }
        }
        // A small ridge keeps coefficients without data well defined.
        for i in 0..n {
            normal.push(i, i, 1e-9);
        }
        let cholesky = CscCholesky::factor(&CscMatrix::from(&normal))
            .expect("normal equations are positive definite");
        let x = cholesky.solve(&rhs).column(0).into_owned();

        let apply = |entries: &[(usize, f64)], v: &DVector<f64>| -> f64 {
            entries.iter().map(|&(i, a)| a * v[i]).sum()
        };
        self.residuals = DVector::from_iterator(
            rows.len(),
            rows.iter()
                .map(|row| weight(row) * (apply(&row.entries, &x) - row.rhs)),
        );

        let latencies = self.latencies.len();
        self.jacobian = DMatrix::zeros(rows.len(), latencies);
        for p in 0..latencies {
            // dr/dτ ≈ (I − A·A⁺)·(∂A/∂τ)·x
            let v = DVector::from_iterator(
                rows.len(),
                rows.iter().map(|row| match &row.d_latency {
                    Some((q, entries)) if *q == p => weight(row) * apply(entries, &x),
                    _ => 0.0,
                }),
            );
            let mut at_v = DVector::zeros(n);
            for (row, &vk) in rows.iter().zip(v.iter()) {
                if vk != 0.0 {
                    for &(i, a) in &row.entries {
                        at_v[i] += weight(row) * a * vk;
                    }
                }
            }
            let y = cholesky.solve(&at_v).column(0).into_owned();
            for (k, row) in rows.iter().enumerate() {
                self.jacobian[(k, p)] = v[k] - weight(row) * apply(&row.entries, &y);
            }
        }
    }
}

impl LeastSquaresProblem<f64, Dyn, Dyn> for Projection<'_> {
    type ResidualStorage = Owned<f64, Dyn>;
    type JacobianStorage = Owned<f64, Dyn, Dyn>;
    type ParameterStorage = Owned<f64, Dyn>;

    fn set_params(&mut self, latencies: &DVector<f64>) {
        self.latencies.copy_from(latencies);
        self.update();
    }

    fn params(&self) -> DVector<f64> {
        self.latencies.clone()
    }

    fn residuals(&self) -> Option<DVector<f64>> {
        Some(self.residuals.clone())
    }

    fn jacobian(&self) -> Option<DMatrix<f64>> {
        Some(self.jacobian.clone())
    }
}
