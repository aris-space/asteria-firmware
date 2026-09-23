//! SEF-light vertical estimation from the calibrated sensor streams.

use asteria_sef_light::{EstimatorError, PressureMeasurement};
use defmt::{Debug2Format, info, warn};
use embassy_time::{Duration, Instant};
use fw_sensor_carrier_v3::sef::{
    Estimator, GNSS_HEIGHT_STD_FLOOR_M, GnssVerticalInput, ImuWindow,
    barometric_pressure_altitude_m, gnss_measurement, imu_measurement, new_estimator,
};

use crate::calibration;
use crate::sensors::{GnssId, IMU_0, IMU_1};
use crate::signals;
use crate::tasks::readout::imu::GYRO_RANGE_DPS;
use crate::types::{BaroSample, GnssSample, ImuSample, MagSample, StateEstimate};

const OUTPUT_PERIOD: Duration = Duration::from_millis(50);
const IMU_FRESH: Duration = Duration::from_millis(100);
const GNSS_FRESH: Duration = Duration::from_millis(500);
const GNSS_FUSION_PERIOD: Duration = Duration::from_secs(1);
const GNSS_MAX_VERTICAL_ACCURACY_MM: u32 = 3_000;
// A four-satellite startup fix reported a misleading 2.7 m vAcc at PDOP 9.4.
const GNSS_MAX_PDOP_CENTI: u16 = 600;
const GNSS_SWITCH_IMPROVEMENT: f32 = 1.5;
const STATE_LOG_PERIOD: Duration = Duration::from_millis(250);
const STATUS_LOG_PERIOD: Duration = Duration::from_secs(1);
const IMU_DIAGNOSTIC_PERIOD: Duration = Duration::from_secs(10);
// Both stationary barometers varied by about 0.3 m in the bench run; this
// larger uncertainty allows for pressure changes and correlated samples.
const BARO_HEIGHT_STD_M: f32 = 1.5;

#[derive(Clone, Copy)]
enum Event {
    Imu(ImuSample),
    Barometer(BaroSample),
    Magnetometer(MagSample),
    Gnss(GnssSample),
}

impl Event {
    fn ts(self) -> Instant {
        match self {
            Self::Imu(sample) => sample.ts,
            Self::Barometer(sample) => sample.ts,
            Self::Magnetometer(sample) => sample.ts,
            Self::Gnss(sample) => sample.ts,
        }
    }
}

struct Processor {
    estimator: Estimator,
    gnss_ready: bool,
    gnss_height_std_m: f32,
    gnss_latest: [Option<GnssSample>; 2],
    selected_gnss: Option<GnssId>,
    last_gnss_fusion: Option<Instant>,
    last_output: Option<Instant>,
    last_status_log: Option<Instant>,
    last_state_log: Option<Instant>,
    last_logged_state: [Option<(f32, f32)>; 2],
    imu_windows: [ImuWindow; 2],
    last_imu_report: Instant,
    last_baro_log: [Option<Instant>; 2],
    last_mag_log: [Option<Instant>; 2],
    last_warning: Option<Instant>,
    published_since_status: u32,
}

impl Processor {
    fn new() -> Result<Self, EstimatorError> {
        Ok(Self {
            estimator: new_estimator(GYRO_RANGE_DPS)?,
            gnss_ready: false,
            gnss_height_std_m: GNSS_HEIGHT_STD_FLOOR_M,
            gnss_latest: [None; 2],
            selected_gnss: None,
            last_gnss_fusion: None,
            last_output: None,
            last_status_log: None,
            last_state_log: None,
            last_logged_state: [None; 2],
            imu_windows: [ImuWindow::default(); 2],
            last_imu_report: Instant::now(),
            last_baro_log: [None; 2],
            last_mag_log: [None; 2],
            last_warning: None,
            published_since_status: 0,
        })
    }

    fn handle(&mut self, event: Event) {
        let result = match event {
            Event::Imu(sample) => self.update_imu(sample),
            Event::Barometer(sample) => self.update_barometer(sample),
            Event::Magnetometer(sample) => self.update_magnetometer(sample),
            Event::Gnss(sample) => self.update_gnss(sample),
        };
        if let Err(error) = result {
            let now = Instant::now();
            if self
                .last_warning
                .is_none_or(|last| now.saturating_duration_since(last) >= STATUS_LOG_PERIOD)
            {
                warn!("SEF-light update failed: {:?}", Debug2Format(&error));
                self.last_warning = Some(now);
            }
        }
        self.publish();
        self.report_imu_diagnostics();
    }

    fn update_imu(&mut self, sample: ImuSample) -> Result<(), EstimatorError> {
        let imu = asteria_sef_light::ImuId::from_index(sample.src.index())
            .expect("firmware IMU ID must map to SEF-light");
        let measurement = imu_measurement(
            [sample.accel.x, sample.accel.y, sample.accel.z],
            [sample.gyro.x, sample.gyro.y, sample.gyro.z],
        );
        self.estimator
            .update_imu(imu, sample.ts.as_micros(), measurement)?;
        self.imu_windows[sample.src.index()].record(measurement);
        Ok(())
    }

    fn update_barometer(&mut self, sample: BaroSample) -> Result<(), EstimatorError> {
        let index = sample.src.index();
        let height_m = barometric_pressure_altitude_m(sample.pressure_mbar)
            .ok_or(EstimatorError::OutOfRangeInput)?;
        if self.last_baro_log[index]
            .is_none_or(|last| sample.ts.saturating_duration_since(last) >= STATUS_LOG_PERIOD)
        {
            info!(
                "SEF baro {}: pressure_altitude={} m, pressure={} mbar, temperature={} C",
                sample.src, height_m, sample.pressure_mbar, sample.temperature_c
            );
            self.last_baro_log[index] = Some(sample.ts);
        }
        let barometer = asteria_sef_light::BarometerId::from_index(index)
            .expect("firmware barometer ID must map to SEF-light");
        self.estimator.update_pressure(
            sample.ts.as_micros(),
            barometer,
            PressureMeasurement {
                height_m,
                height_std_m: BARO_HEIGHT_STD_M,
            },
        )?;
        Ok(())
    }

    fn update_magnetometer(&mut self, sample: MagSample) -> Result<(), EstimatorError> {
        let index = sample.src.index();
        let field = [sample.x, sample.y, sample.z];
        let field_nt = libm::sqrtf(field.iter().map(|value| value * value).sum());
        let calibration = calibration::mag::applied()[index];
        if self.last_mag_log[index]
            .is_none_or(|last| sample.ts.saturating_duration_since(last) >= STATUS_LOG_PERIOD)
        {
            info!(
                "SEF mag {}: field={} nT, calibrated={}, accepted={}",
                sample.src,
                field_nt,
                calibration.is_calibrated(),
                calibration.accepts_field(field_nt)
            );
            self.last_mag_log[index] = Some(sample.ts);
        }
        if !calibration.accepts_field(field_nt) {
            return Ok(());
        }
        let imu = asteria_sef_light::ImuId::from_index(index)
            .expect("firmware magnetometer ID must map to SEF-light IMU");
        self.estimator
            .update_magnetometer(imu, sample.ts.as_micros(), field)
    }

    fn update_gnss(&mut self, sample: GnssSample) -> Result<(), EstimatorError> {
        let Some(best) = self.select_gnss(sample) else {
            return Ok(());
        };
        if !self.gnss_ready {
            // An unanchored barometer/IMU startup can leave the vertical state
            // too far from MSL for the innovation gate to accept the first fix.
            self.estimator = new_estimator(GYRO_RANGE_DPS)?;
            self.last_logged_state = [None; 2];
        }
        // NavPVT arrives at 20 Hz, but adjacent GNSS heights are strongly
        // correlated. Fuse them at 1 Hz while retaining the full receiver rate.
        if self
            .last_gnss_fusion
            .is_some_and(|last| sample.ts.saturating_duration_since(last) < GNSS_FUSION_PERIOD)
        {
            return Ok(());
        }
        let mut measurements = [None, None];
        measurements[best.src.index()] = Some(gnss_measurement(GnssVerticalInput {
            height_msl_m: best.pvt.height_msl,
            velocity_down_mps: best.pvt.vel_down,
            vertical_accuracy_mm: best.pvt.vert_accuracy,
            speed_accuracy_mps: best.pvt.speed_accuracy_mps,
            fix_tier: 3,
            pdop_centi: best.pvt.pdop,
        }));
        let height_std_m = measurements[best.src.index()]
            .expect("GNSS measurement set")
            .measurement
            .height_std_m;
        if let Some(updates) = self
            .estimator
            .update_gnss(sample.ts.as_micros(), measurements)?
        {
            let selected = &updates[self.estimator.selected_imu().index()];
            info!(
                "SEF GNSS update {}: h accepted={}, innovation={} m, nis={}, v accepted={}, innovation={} m/s, nis={}",
                best.src,
                selected.height.accepted,
                selected.height.innovation,
                selected.height.normalized_innovation_squared,
                selected.velocity.accepted,
                selected.velocity.innovation,
                selected.velocity.normalized_innovation_squared,
            );
            if selected.height.accepted {
                self.gnss_ready = true;
                self.gnss_height_std_m = height_std_m;
            }
            self.last_gnss_fusion = Some(sample.ts);
        }
        Ok(())
    }

    fn select_gnss(&mut self, sample: GnssSample) -> Option<GnssSample> {
        // A poor 3D fix can be tens of metres from a later precise fix. Apply
        // the same quality requirement to both receivers at every update.
        let valid = sample.pvt.height_msl.is_finite()
            && sample.pvt.vert_accuracy <= GNSS_MAX_VERTICAL_ACCURACY_MM
            && sample.pvt.pdop <= GNSS_MAX_PDOP_CENTI
            && matches!(
                sample.pvt.fix_type,
                ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning
            );
        if !valid {
            self.gnss_latest[sample.src.index()] = None;
            return None;
        }
        self.gnss_latest[sample.src.index()] = Some(sample);
        let now = Instant::now();
        let [first, second] = self.gnss_latest.map(|candidate| {
            candidate.filter(|candidate| now.saturating_duration_since(candidate.ts) <= GNSS_FRESH)
        });
        // Vertical accuracy estimates height error directly. Keep the current
        // fresh receiver until another reports at least 1.5x better accuracy.
        let candidate = match (first, second) {
            (Some(first), Some(second)) => {
                if first.pvt.vert_accuracy < second.pvt.vert_accuracy {
                    first
                } else {
                    second
                }
            }
            (Some(first), None) => first,
            (None, Some(second)) => second,
            (None, None) => return None,
        };
        let best = match self
            .selected_gnss
            .and_then(|source| [first, second][source.index()])
        {
            Some(current)
                if current.src != candidate.src
                    && (candidate.pvt.vert_accuracy as f32) * GNSS_SWITCH_IMPROVEMENT
                        >= current.pvt.vert_accuracy as f32 =>
            {
                current
            }
            _ => candidate,
        };
        if sample.src != best.src {
            return None;
        }
        if self.selected_gnss != Some(best.src) {
            info!(
                "SEF GNSS source: {} (vAcc={} mm, PDOP={})",
                best.src, best.pvt.vert_accuracy, best.pvt.pdop
            );
            self.selected_gnss = Some(best.src);
            self.last_gnss_fusion = None;
        }
        Some(best)
    }

    fn publish(&mut self) {
        let now = Instant::now();
        if self
            .last_output
            .is_some_and(|last| now.saturating_duration_since(last) < OUTPUT_PERIOD)
        {
            return;
        }
        let imu = self.estimator.selected_imu();
        let Some(sample_time_us) = self.estimator.last_imu_sample_time_us(imu) else {
            return;
        };
        let ts = Instant::from_micros(sample_time_us);
        if now.saturating_duration_since(ts) > IMU_FRESH || !self.estimator.imu_ready(imu) {
            return;
        }
        // MSL comes from GNSS. The AHRS orientation is useful before the
        // barometer bias states have an absolute height reference.
        let state = self.estimator.selected_state();
        let uncertainty = self.estimator.selected_uncertainty();
        let altitude_msl_m = state.height_m;
        let height_std_m = libm::sqrtf(uncertainty.height_variance_m2).max(self.gnss_height_std_m);
        let selected_imu = if imu.index() == 0 { IMU_0 } else { IMU_1 };
        signals::STATE_ESTIMATE_WATCH.sender().send(StateEstimate {
            ts,
            msl_ready: self.gnss_ready,
            height_msl_m: altitude_msl_m,
            velocity_mps: state.velocity_mps,
            height_std_m,
            velocity_std_mps: libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
            orientation_body_to_ned_wxyz: self.estimator.orientation_body_to_ned_wxyz(imu),
            selected_imu,
            redundancy_ready: self.estimator.redundancy_ready(),
        });
        self.published_since_status = self.published_since_status.saturating_add(1);
        self.log_full_state(now);
        if self
            .last_status_log
            .is_none_or(|last| now.saturating_duration_since(last) >= STATUS_LOG_PERIOD)
        {
            if self.gnss_ready {
                info!(
                    "SEF-light: altitude_msl={}±{} m, v={}±{} m/s, IMU={}, redundancy_ready={}, published={}",
                    altitude_msl_m,
                    height_std_m,
                    state.velocity_mps,
                    libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
                    selected_imu,
                    self.estimator.redundancy_ready(),
                    self.published_since_status,
                );
            } else {
                info!("SEF-light: waiting for usable GNSS fix to establish MSL altitude");
            }
            self.published_since_status = 0;
            self.last_status_log = Some(now);
        }
        self.last_output = Some(now);
    }

    fn log_full_state(&mut self, now: Instant) {
        if self
            .last_state_log
            .is_some_and(|last| now.saturating_duration_since(last) < STATE_LOG_PERIOD)
        {
            return;
        }
        let scores = self.estimator.consistency_scores();
        for (index, id) in [IMU_0, IMU_1].into_iter().enumerate() {
            let imu = asteria_sef_light::ImuId::from_index(index)
                .expect("firmware IMU ID must map to SEF-light");
            let state = self.estimator.state(imu);
            let uncertainty = self.estimator.uncertainty(imu);
            let attitude = self.estimator.imu_status(imu);
            let q = self.estimator.orientation_body_to_ned_wxyz(imu);
            let previous =
                self.last_logged_state[index].unwrap_or((state.height_m, state.velocity_mps));
            info!(
                "SEF {}: h={} dh={} m, v={} dv={} m/s, bias=[{},{}] m, std=[{},{},{},{}], cov_hv={}, score={}, q=[{},{},{},{}], attitude_error={} rad, ignored={}, mag_error={} rad, mag_ignored={}, flags=[{},{},{}]",
                id,
                state.height_m,
                state.height_m - previous.0,
                state.velocity_mps,
                state.velocity_mps - previous.1,
                state.barometer_bias_m[0],
                state.barometer_bias_m[1],
                libm::sqrtf(uncertainty.height_variance_m2),
                libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
                libm::sqrtf(uncertainty.barometer_bias_variance_m2[0]),
                libm::sqrtf(uncertainty.barometer_bias_variance_m2[1]),
                uncertainty.height_velocity_covariance_m2_per_s,
                scores[index],
                q[0],
                q[1],
                q[2],
                q[3],
                attitude.acceleration_error_rad,
                attitude.accelerometer_ignored,
                attitude.magnetic_error_rad,
                attitude.magnetometer_ignored,
                attitude.flags.initialising(),
                attitude.flags.angular_rate_recovery(),
                attitude.flags.acceleration_recovery(),
            );
            self.last_logged_state[index] = Some((state.height_m, state.velocity_mps));
        }
        self.last_state_log = Some(now);
    }

    fn report_imu_diagnostics(&mut self) {
        let now = Instant::now();
        if now.saturating_duration_since(self.last_imu_report) < IMU_DIAGNOSTIC_PERIOD {
            return;
        }
        for (index, id) in [IMU_0, IMU_1].into_iter().enumerate() {
            let Some(summary) = core::mem::take(&mut self.imu_windows[index]).summary() else {
                continue;
            };
            // This drift proxy applies only when the carrier is stationary and
            // the gravity-norm error projects onto the vertical axis.
            let window_s = IMU_DIAGNOSTIC_PERIOD.as_secs() as f32;
            let free_height_drift_m = 0.5 * window_s * window_s * summary.gravity_error_mean_mps2;
            info!(
                "IMU {} bench: n={}, gravity_error_mean={} m/s2, gravity_noise={} m/s2, max_accel={} m/s2, gyro_mean=[{},{},{}] rad/s, gyro_noise={} rad/s, max_gyro={} rad/s, free_dh_10s={} m",
                id,
                summary.samples,
                summary.gravity_error_mean_mps2,
                summary.gravity_error_noise_mps2,
                summary.max_acceleration_mps2,
                summary.gyro_mean_rad_s[0],
                summary.gyro_mean_rad_s[1],
                summary.gyro_mean_rad_s[2],
                summary.gyro_noise_rad_s,
                summary.max_gyro_rad_s,
                free_height_drift_m,
            );
        }
        self.last_imu_report = now;
    }
}

mod stream;
pub use stream::task;
