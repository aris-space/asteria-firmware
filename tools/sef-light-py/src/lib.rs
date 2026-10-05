//! Python bindings to [`asteria_sef_light::DualVerticalEstimator`], one method
//! per estimator method, so SD logs can be replayed with other settings.

use asteria_sef_light::{
    BarometerId, DualVerticalEstimator, EstimatorError, GnssSample, GnssSelectorConfig,
    ImuAttitudeConfig, ImuId, ImuMeasurement, PressureMeasurement, SelectorConfig,
    VerticalEstimatorSelectorConfig, VerticalFilterConfig, VerticalGnssMeasurement,
};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

// The firmware's history capacity.
const HISTORY_CAPACITY: usize = 768;

fn error(error: EstimatorError) -> PyErr {
    PyValueError::new_err(format!("{error:?}"))
}

fn imu(index: usize) -> PyResult<ImuId> {
    ImuId::ALL
        .get(index)
        .copied()
        .ok_or_else(|| PyValueError::new_err("IMU index out of range"))
}

#[pyclass]
struct Estimator(DualVerticalEstimator<HISTORY_CAPACITY>);

#[pymethods]
impl Estimator {
    #[new]
    #[pyo3(signature = (
        *,
        acceleration_noise_std_mps2,
        degraded_acceleration_noise_std_mps2,
        barometer_bias_walk_std_m_per_sqrt_s,
        initial_height_std_m,
        initial_velocity_std_mps,
        initial_barometer_bias_std_m,
        innovation_gate_sigma,
        ahrs_gain,
        gyroscope_range_deg_s,
        acceleration_rejection_deg,
        recovery_trigger_period,
        magnetic_rejection_deg,
        switch_hysteresis,
        switch_dwell_us,
        score_memory,
        maximum_nis_contribution,
        degraded_score_penalty,
        maximum_imu_age_us,
        gnss_minimum_fix_tier,
        gnss_consistency_gate_sigma,
        gnss_switch_dwell_us,
        maximum_aiding_delay_us,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        acceleration_noise_std_mps2: f32,
        degraded_acceleration_noise_std_mps2: f32,
        barometer_bias_walk_std_m_per_sqrt_s: [f32; 2],
        initial_height_std_m: f32,
        initial_velocity_std_mps: f32,
        initial_barometer_bias_std_m: [f32; 2],
        innovation_gate_sigma: f32,
        ahrs_gain: f32,
        gyroscope_range_deg_s: f32,
        acceleration_rejection_deg: f32,
        recovery_trigger_period: u32,
        magnetic_rejection_deg: f32,
        switch_hysteresis: f32,
        switch_dwell_us: u64,
        score_memory: f32,
        maximum_nis_contribution: f32,
        degraded_score_penalty: f32,
        maximum_imu_age_us: u64,
        gnss_minimum_fix_tier: u8,
        gnss_consistency_gate_sigma: f32,
        gnss_switch_dwell_us: u64,
        maximum_aiding_delay_us: u64,
    ) -> PyResult<Self> {
        let out_of_range = || error(EstimatorError::OutOfRangeInput);
        let filter = VerticalFilterConfig::new(
            acceleration_noise_std_mps2,
            degraded_acceleration_noise_std_mps2,
            barometer_bias_walk_std_m_per_sqrt_s,
            initial_height_std_m,
            initial_velocity_std_mps,
            initial_barometer_bias_std_m,
            innovation_gate_sigma,
        )
        .map_err(error)?;
        let attitude = ImuAttitudeConfig::new(
            ahrs_gain,
            gyroscope_range_deg_s,
            acceleration_rejection_deg,
            recovery_trigger_period,
        )
        .and_then(|config| config.with_magnetic_rejection(magnetic_rejection_deg))
        .map_err(error)?;
        let selection =
            SelectorConfig::new(switch_hysteresis, switch_dwell_us).ok_or_else(out_of_range)?;
        let selector = VerticalEstimatorSelectorConfig::new(
            score_memory,
            maximum_nis_contribution,
            degraded_score_penalty,
            maximum_imu_age_us,
            selection,
        )
        .map_err(error)?;
        let gnss = GnssSelectorConfig::new(
            gnss_minimum_fix_tier,
            gnss_consistency_gate_sigma,
            gnss_switch_dwell_us,
        )
        .ok_or_else(out_of_range)?;
        DualVerticalEstimator::new(
            filter,
            [attitude; 2],
            selector,
            gnss,
            maximum_aiding_delay_us,
        )
        .map(Self)
        .map_err(error)
    }

    /// Returns whether the IMU's chain advanced.
    fn update_imu(
        &mut self,
        imu_index: usize,
        sample_time_us: u64,
        acceleration_body_mps2: [f32; 3],
        angular_rate_body_rad_s: [f32; 3],
    ) -> PyResult<bool> {
        let measurement = ImuMeasurement {
            acceleration_body_mps2,
            angular_rate_body_rad_s,
        };
        self.0
            .update_imu(imu(imu_index)?, sample_time_us, measurement)
            .map_err(error)
    }

    fn update_magnetometer(
        &mut self,
        imu_index: usize,
        sample_time_us: u64,
        field: [f32; 3],
    ) -> PyResult<()> {
        self.0
            .update_magnetometer(imu(imu_index)?, sample_time_us, field)
            .map_err(error)
    }

    /// Returns per chain whether the measurement was accepted.
    fn update_pressure(
        &mut self,
        sample_time_us: u64,
        barometer_index: usize,
        height_m: f32,
        height_std_m: f32,
    ) -> PyResult<[bool; 2]> {
        let barometer = *BarometerId::ALL
            .get(barometer_index)
            .ok_or_else(|| PyValueError::new_err("barometer index out of range"))?;
        let measurement = PressureMeasurement {
            height_m,
            height_std_m,
        };
        self.0
            .update_pressure(sample_time_us, barometer, measurement)
            .map(|updates| updates.map(|update| update.accepted))
            .map_err(error)
    }

    /// Fuses one receiver's solution. Returns per chain whether the height
    /// was accepted, or `None` if the selector did not use the solution.
    #[allow(clippy::too_many_arguments)]
    fn update_gnss(
        &mut self,
        fusion_time_us: u64,
        receiver_index: usize,
        height_m: f32,
        velocity_mps: f32,
        height_std_m: f32,
        velocity_std_mps: f32,
        fix_tier: u8,
        pdop_centi: u16,
    ) -> PyResult<Option<[bool; 2]>> {
        let mut samples = [None; 2];
        *samples
            .get_mut(receiver_index)
            .ok_or_else(|| PyValueError::new_err("receiver index out of range"))? =
            Some(GnssSample {
                measurement: VerticalGnssMeasurement {
                    height_m,
                    velocity_mps,
                    height_std_m,
                    velocity_std_mps,
                },
                fix_tier,
                pdop_centi,
            });
        self.0
            .update_gnss(fusion_time_us, samples)
            .map(|updates| updates.map(|updates| updates.map(|update| update.height.accepted)))
            .map_err(error)
    }

    /// `(height_m, velocity_mps, [barometer_bias_m; 2])` of one chain.
    fn state(&self, imu_index: usize) -> PyResult<(f32, f32, [f32; 2])> {
        let state = self.0.state(imu(imu_index)?);
        Ok((state.height_m, state.velocity_mps, state.barometer_bias_m))
    }

    /// `(height_var_m2, velocity_var_m2_s2, [barometer_bias_var_m2; 2])` of one chain.
    fn uncertainty(&self, imu_index: usize) -> PyResult<(f32, f32, [f32; 2])> {
        let u = self.0.uncertainty(imu(imu_index)?);
        Ok((
            u.height_variance_m2,
            u.velocity_variance_m2_per_s2,
            u.barometer_bias_variance_m2,
        ))
    }

    fn orientation_body_to_ned_wxyz(&self, imu_index: usize) -> PyResult<[f32; 4]> {
        Ok(self.0.orientation_body_to_ned_wxyz(imu(imu_index)?))
    }

    fn selected_imu(&self) -> usize {
        self.0.selected_imu().index()
    }

    fn consistency_scores(&self) -> [f32; 2] {
        self.0.consistency_scores()
    }

    fn imu_ready(&self, imu_index: usize) -> PyResult<bool> {
        Ok(self.0.imu_ready(imu(imu_index)?))
    }

    fn redundancy_ready(&self) -> bool {
        self.0.redundancy_ready()
    }
}

#[pymodule]
fn sef_light(module: &Bound<'_, PyModule>) -> PyResult<()> {
    module.add_class::<Estimator>()
}
