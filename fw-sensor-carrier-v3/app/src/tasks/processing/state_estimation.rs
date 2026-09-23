//! SEF-light vertical estimation from the calibrated sensor streams.

use asteria_sef_light::{EstimatorError, PressureMeasurement};
use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, Either6, select, select6};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer};
use fw_sensor_carrier_v3::sef::{
    Estimator, GNSS_HEIGHT_STD_FLOOR_M, GnssVerticalInput, ImuWindow,
    barometric_pressure_altitude_m, gnss_measurement, imu_measurement, new_estimator,
};

use crate::sensors::{BARO_BUS_1, BARO_BUS_2, GNSS_0, GNSS_1, GnssId, IMU_0, IMU_1};
use crate::signals;
use crate::tasks::readout::imu::GYRO_RANGE_DPS;
use crate::types::{BaroSample, GnssSample, ImuSample, VerticalEstimate};

const OUTPUT_PERIOD: Duration = Duration::from_millis(50);
const IMU_FRESH: Duration = Duration::from_millis(100);
const GNSS_FRESH: Duration = Duration::from_millis(500);
const GNSS_FUSION_PERIOD: Duration = Duration::from_secs(1);
const STATE_LOG_PERIOD: Duration = Duration::from_millis(250);
const IMU_DIAGNOSTIC_PERIOD: Duration = Duration::from_secs(10);
const IMU_MERGE_HOLDBACK: Duration = Duration::from_millis(35);
const BARO_HEIGHT_STD_M: f32 = 3.0;

enum Event {
    Imu(ImuSample),
    Barometer(BaroSample),
    Gnss(GnssSample),
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
            last_warning: None,
            published_since_status: 0,
        })
    }

    fn handle(&mut self, event: Event) {
        let result = match event {
            Event::Imu(sample) => self.update_imu(sample),
            Event::Barometer(sample) => self.update_barometer(sample),
            Event::Gnss(sample) => self.update_gnss(sample),
        };
        if let Err(error) = result {
            let now = Instant::now();
            if self
                .last_warning
                .is_none_or(|last| now.saturating_duration_since(last) >= Duration::from_secs(1))
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
            .is_none_or(|last| sample.ts.saturating_duration_since(last) >= Duration::from_secs(1))
        {
            info!(
                "SEF baro {}: pressure_altitude={} m, pressure={} mbar",
                sample.src, height_m, sample.pressure_mbar
            );
            self.last_baro_log[index] = Some(sample.ts);
        }
        // MSL comes from GNSS. Before its first fix, an uncalibrated pressure
        // altitude cannot establish either the height or barometer biases.
        if !self.gnss_ready {
            return Ok(());
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

    fn update_gnss(&mut self, sample: GnssSample) -> Result<(), EstimatorError> {
        let valid = sample.pvt.height_msl.is_finite()
            && matches!(
                sample.pvt.fix_type,
                ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning
            );
        if !valid {
            self.gnss_latest[sample.src.index()] = None;
            return Ok(());
        }
        self.gnss_latest[sample.src.index()] = Some(sample);
        let now = Instant::now();
        let [first, second] = self.gnss_latest.map(|candidate| {
            candidate.filter(|candidate| now.saturating_duration_since(candidate.ts) <= GNSS_FRESH)
        });
        let best = match (first, second) {
            (Some(first), Some(second)) => {
                if first.pvt.vert_accuracy < second.pvt.vert_accuracy {
                    first
                } else {
                    second
                }
            }
            (Some(first), None) => first,
            (None, Some(second)) => second,
            (None, None) => return Ok(()),
        };
        if sample.src != best.src {
            return Ok(());
        }
        if self.selected_gnss != Some(best.src) {
            info!(
                "SEF GNSS source: {} (vAcc={} mm)",
                best.src, best.pvt.vert_accuracy
            );
            self.selected_gnss = Some(best.src);
            self.last_gnss_fusion = None;
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
            height_msl_m: sample.pvt.height_msl,
            velocity_down_mps: sample.pvt.vel_down,
            vertical_accuracy_mm: sample.pvt.vert_accuracy,
            speed_accuracy_mps: sample.pvt.speed_accuracy_mps,
            fix_tier: 3,
            pdop_centi: sample.pvt.pdop,
        }));
        let height_std_m = measurements[best.src.index()]
            .expect("GNSS measurement set")
            .measurement
            .height_std_m;
        if let Some(updates) = self
            .estimator
            .update_gnss(sample.ts.as_micros(), measurements)?
        {
            if updates.iter().any(|update| update.height.accepted) {
                self.gnss_ready = true;
                self.gnss_height_std_m = height_std_m;
            }
            self.last_gnss_fusion = Some(sample.ts);
        }
        Ok(())
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
        if !self.gnss_ready {
            if self
                .last_status_log
                .is_none_or(|last| now.saturating_duration_since(last) >= Duration::from_secs(1))
            {
                info!("SEF-light: waiting for GNSS 3D fix to establish MSL altitude");
                self.last_status_log = Some(now);
            }
            return;
        }
        let state = self.estimator.selected_state();
        let uncertainty = self.estimator.selected_uncertainty();
        let altitude_msl_m = state.height_m;
        let height_std_m = libm::sqrtf(uncertainty.height_variance_m2).max(self.gnss_height_std_m);
        let selected_imu = if imu.index() == 0 { IMU_0 } else { IMU_1 };
        signals::VERTICAL_ESTIMATE_WATCH
            .sender()
            .send(VerticalEstimate {
                ts,
                height_msl_m: altitude_msl_m,
                velocity_mps: state.velocity_mps,
                height_std_m,
                velocity_std_mps: libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
                selected_imu,
                redundancy_ready: self.estimator.redundancy_ready(),
            });
        self.published_since_status = self.published_since_status.saturating_add(1);
        self.log_full_state(now);
        if self
            .last_status_log
            .is_none_or(|last| now.saturating_duration_since(last) >= Duration::from_secs(1))
        {
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
                "SEF {}: h={} dh={} m, v={} dv={} m/s, bias=[{},{}] m, std=[{},{},{},{}], cov_hv={}, score={}, q=[{},{},{},{}], attitude_error={} rad, ignored={}, flags=[{},{},{}]",
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
            let free_height_drift_10s_m = 50.0 * summary.gravity_error_mean_mps2;
            info!(
                "IMU {} bench: n={}, gravity_error_mean={} m/s2, gravity_noise={} m/s2, gyro_mean=[{},{},{}] rad/s, gyro_noise={} rad/s, free_dh_10s={} m, category={}",
                id,
                summary.samples,
                summary.gravity_error_mean_mps2,
                summary.gravity_error_noise_mps2,
                summary.gyro_mean_rad_s[0],
                summary.gyro_mean_rad_s[1],
                summary.gyro_mean_rad_s[2],
                summary.gyro_noise_rad_s,
                free_height_drift_10s_m,
                summary.category(),
            );
        }
        self.last_imu_report = now;
    }
}

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut imu0 = signals::IMU_CHANNELS[IMU_0.index()]
        .subscriber()
        .expect("SEF IMU 0 subscriber");
    let mut imu1 = signals::IMU_CHANNELS[IMU_1.index()]
        .subscriber()
        .expect("SEF IMU 1 subscriber");
    let mut baro0 = signals::BARO_CHANNELS[BARO_BUS_1.index()]
        .subscriber()
        .expect("SEF barometer 0 subscriber");
    let mut baro1 = signals::BARO_CHANNELS[BARO_BUS_2.index()]
        .subscriber()
        .expect("SEF barometer 1 subscriber");
    let mut gnss0 = signals::GNSS_CHANNELS[GNSS_0.index()]
        .subscriber()
        .expect("SEF GNSS 0 subscriber");
    let mut gnss1 = signals::GNSS_CHANNELS[GNSS_1.index()]
        .subscriber()
        .expect("SEF GNSS 1 subscriber");
    let mut processor = Processor::new().expect("SEF-light configuration must be valid");
    let mut pending_imu: [Option<ImuSample>; 2] = [None, None];
    let mut dropped_imu = [0_u64; 2];
    let mut late_imu = 0_u32;
    let mut last_imu_time: Option<Instant> = None;
    let mut max_imu_backlog = [0_u64; 2];
    let mut queue_report_at = Instant::now() + IMU_DIAGNOSTIC_PERIOD;

    loop {
        max_imu_backlog[0] = max_imu_backlog[0].max(imu0.available());
        max_imu_backlog[1] = max_imu_backlog[1].max(imu1.available());

        if pending_imu[0].is_none() {
            match imu0.try_next_message() {
                Some(WaitResult::Message(sample)) => pending_imu[0] = Some(sample),
                Some(WaitResult::Lagged(count)) => dropped_imu[0] += count,
                None => {}
            }
        }
        if pending_imu[1].is_none() {
            match imu1.try_next_message() {
                Some(WaitResult::Message(sample)) => pending_imu[1] = Some(sample),
                Some(WaitResult::Lagged(count)) => dropped_imu[1] += count,
                None => {}
            }
        }

        // FIFO batches are published separately. Feed their samples in time
        // order so SEF does not replay its whole history for every other IMU.
        let ready_imu = match pending_imu {
            [Some(first), Some(second)] => Some(if first.ts <= second.ts { 0 } else { 1 }),
            [Some(sample), None]
                if Instant::now().saturating_duration_since(sample.ts) >= IMU_MERGE_HOLDBACK =>
            {
                Some(0)
            }
            [None, Some(sample)]
                if Instant::now().saturating_duration_since(sample.ts) >= IMU_MERGE_HOLDBACK =>
            {
                Some(1)
            }
            _ => None,
        };
        if let Some(index) = ready_imu {
            let sample = pending_imu[index].take().expect("ready IMU sample");
            if last_imu_time.is_some_and(|last| sample.ts < last) {
                late_imu = late_imu.saturating_add(1);
            }
            last_imu_time = Some(sample.ts);
            processor.handle(Event::Imu(sample));
            if Instant::now() >= queue_report_at {
                info!(
                    "SEF IMU queues/10s: dropped=[{},{}], late={}, max_backlog=[{},{}]",
                    dropped_imu[0],
                    dropped_imu[1],
                    late_imu,
                    max_imu_backlog[0],
                    max_imu_backlog[1],
                );
                dropped_imu = [0; 2];
                late_imu = 0;
                max_imu_backlog = [0; 2];
                queue_report_at = Instant::now() + IMU_DIAGNOSTIC_PERIOD;
            }
            continue;
        }

        let waiting_imu0 = pending_imu[0].is_some();
        let waiting_imu1 = pending_imu[1].is_some();
        let pending_deadline = pending_imu[0]
            .or(pending_imu[1])
            .map(|sample| sample.ts + IMU_MERGE_HOLDBACK);
        let next = select6(
            baro0.next_message_pure(),
            baro1.next_message_pure(),
            gnss0.next_message_pure(),
            gnss1.next_message_pure(),
            async {
                if waiting_imu0 {
                    core::future::pending().await
                } else {
                    imu0.next_message().await
                }
            },
            async {
                if waiting_imu1 {
                    core::future::pending().await
                } else {
                    imu1.next_message().await
                }
            },
        );
        let timeout = async {
            if let Some(deadline) = pending_deadline {
                Timer::at(deadline).await;
            } else {
                core::future::pending::<()>().await;
            }
        };
        match select(next, timeout).await {
            Either::First(Either6::First(sample) | Either6::Second(sample)) => {
                processor.handle(Event::Barometer(sample));
            }
            Either::First(Either6::Third(sample) | Either6::Fourth(sample)) => {
                processor.handle(Event::Gnss(sample));
            }
            Either::First(Either6::Fifth(WaitResult::Message(sample))) => {
                pending_imu[0] = Some(sample);
            }
            Either::First(Either6::Sixth(WaitResult::Message(sample))) => {
                pending_imu[1] = Some(sample);
            }
            Either::First(Either6::Fifth(WaitResult::Lagged(count))) => {
                dropped_imu[0] += count;
            }
            Either::First(Either6::Sixth(WaitResult::Lagged(count))) => {
                dropped_imu[1] += count;
            }
            Either::Second(()) => {}
        }
    }
}
