//! SEF-light vertical estimation from the calibrated sensor streams.
//!
//! Barometers give height through per-chain bias states; GNSS fits those
//! biases to MSL. Each output tick publishes the selected chain to
//! [`signals::STATE_ESTIMATE_WATCH`] and logs every chain to SD.

use asteria_sef_light::{BarometerBiasMeasurement, EstimatorError, PressureMeasurement};
use defmt::{Debug2Format, warn};
use embassy_time::{Duration, Instant};
use fw_sensor_carrier_v3::sef::{
    BarometerBiasTracker, Estimator, GNSS_HEIGHT_STD_FLOOR_M, barometric_pressure_altitude_m,
    correlated_gnss_height_std_m, imu_measurement, new_estimator,
};

use crate::calibration;
use crate::sensors::{BAROMETER_COUNT, GnssId, IMU_COUNT, ImuId};
use crate::signals;
use crate::tasks::readout::imu::GYRO_RANGE_DPS;
use crate::types::{
    BaroSample, GnssSample, ImuSample, MagSample, SdLogRecord, SefLogSample, StateEstimate,
};
use gnss_selection::GnssSelection;

const OUTPUT_PERIOD: Duration = Duration::from_millis(50);
const IMU_FRESH: Duration = Duration::from_millis(100);
const GNSS_FUSION_PERIOD: Duration = Duration::from_millis(45);
const GNSS_VELOCITY_FRESH: Duration = Duration::from_secs(2);
const BARO_FOR_GNSS_MAX_AGE: Duration = Duration::from_millis(100);
const WARNING_PERIOD: Duration = Duration::from_secs(1);
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
    gnss: GnssSelection,
    /// Time of the first GNSS bias fit accepted by the selected chain.
    anchored_at: Option<Instant>,
    /// Per chain: a GNSS bias fit was accepted, and a pressure update followed it.
    /// A later IMU handover must not publish an unanchored chain as MSL.
    bias_anchored: [bool; IMU_COUNT],
    pressure_after_anchor: [bool; IMU_COUNT],
    gnss_height_std_m: f32,
    last_gnss_fusion: Option<(GnssId, Instant)>,
    last_gnss_velocity: Option<(Instant, f32)>,
    gnss_displacement_m: f32,
    barometer_bias_tracker: BarometerBiasTracker,
    last_baro_height: [Option<(Instant, f32)>; BAROMETER_COUNT],
    last_output: Option<Instant>,
    last_warning: Option<Instant>,
}

impl Processor {
    fn new() -> Result<Self, EstimatorError> {
        Ok(Self {
            estimator: new_estimator(GYRO_RANGE_DPS)?,
            gnss: GnssSelection::default(),
            anchored_at: None,
            bias_anchored: [false; IMU_COUNT],
            pressure_after_anchor: [false; IMU_COUNT],
            gnss_height_std_m: GNSS_HEIGHT_STD_FLOOR_M,
            last_gnss_fusion: None,
            last_gnss_velocity: None,
            gnss_displacement_m: 0.0,
            barometer_bias_tracker: BarometerBiasTracker::default(),
            last_baro_height: [None; BAROMETER_COUNT],
            last_output: None,
            last_warning: None,
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
                .is_none_or(|last| now.saturating_duration_since(last) >= WARNING_PERIOD)
            {
                warn!("SEF-light update failed: {:?}", Debug2Format(&error));
                self.last_warning = Some(now);
            }
        }
        self.publish();
    }

    fn update_imu(&mut self, sample: ImuSample) -> Result<(), EstimatorError> {
        let measurement = imu_measurement(
            [sample.accel.x, sample.accel.y, sample.accel.z],
            [sample.gyro.x, sample.gyro.y, sample.gyro.z],
        );
        self.estimator.update_imu(
            sef_imu(sample.src.index()),
            sample.ts.as_micros(),
            measurement,
        )?;
        Ok(())
    }

    fn update_barometer(&mut self, sample: BaroSample) -> Result<(), EstimatorError> {
        let index = sample.src.index();
        let height_m = barometric_pressure_altitude_m(sample.pressure_mbar)
            .ok_or(EstimatorError::OutOfRangeInput)?;
        let barometer = asteria_sef_light::BarometerId::from_index(index)
            .expect("firmware barometer ID must map to SEF-light");
        let updates = self.estimator.update_pressure(
            sample.ts.as_micros(),
            barometer,
            PressureMeasurement {
                height_m,
                height_std_m: BARO_HEIGHT_STD_M,
            },
        )?;
        self.last_baro_height[index] = Some((sample.ts, height_m));
        if self.anchored_at.is_some_and(|anchor| sample.ts > anchor) {
            for (ready, update) in self.pressure_after_anchor.iter_mut().zip(updates) {
                *ready |= update.accepted;
            }
        }

        let gnss_velocity_fresh = self.last_gnss_velocity.is_some_and(|(last, _)| {
            sample.ts.saturating_duration_since(last) <= GNSS_VELOCITY_FRESH
        });
        if self.anchored_at.is_none() || !gnss_velocity_fresh {
            self.barometer_bias_tracker.reset_windows();
            return Ok(());
        }
        if let Some(walk_std) = self.barometer_bias_tracker.observe(
            index,
            sample.ts.as_micros(),
            height_m,
            self.gnss_displacement_m,
        ) {
            self.estimator
                .set_barometer_bias_walk_std(barometer, walk_std)?;
        }
        Ok(())
    }

    fn update_magnetometer(&mut self, sample: MagSample) -> Result<(), EstimatorError> {
        let index = sample.src.index();
        let field = [sample.x, sample.y, sample.z];
        let field_nt = libm::sqrtf(field.iter().map(|value| value * value).sum());
        if !calibration::mag::applied()[index].accepts_field(field_nt) {
            return Ok(());
        }
        // Each magnetometer aids the attitude chain of the IMU with the same index.
        self.estimator
            .update_magnetometer(sef_imu(index), sample.ts.as_micros(), field)
    }

    fn update_gnss(&mut self, sample: GnssSample) -> Result<(), EstimatorError> {
        if !self.gnss.select(sample) {
            return Ok(());
        }
        // Fuse every NavPVT epoch, up to the receiver's 20 Hz rate. Adjacent
        // heights are correlated; scale their information by the actual
        // interval so a 1 Hz receiver is not weakened like a 20 Hz one.
        let since_last_fusion = self
            .last_gnss_fusion
            .filter(|&(source, _)| source == sample.src)
            .map(|(_, last)| sample.ts.saturating_duration_since(last));
        if since_last_fusion.is_some_and(|elapsed| elapsed < GNSS_FUSION_PERIOD) {
            return Ok(());
        }
        let height_std_m = self.gnss.height_std_m(&sample);
        let interval_us = since_last_fusion.map_or(1_000_000, |elapsed| elapsed.as_micros());
        let bias_std_m = libm::hypotf(
            correlated_gnss_height_std_m(height_std_m, interval_us),
            BARO_HEIGHT_STD_M,
        );

        let measurements =
            self.barometer_bias_measurements(sample.ts, sample.pvt.height_msl, bias_std_m);
        if measurements.iter().any(Option::is_some) {
            if self.anchored_at.is_none() {
                self.restart_for_first_anchor()?;
            }
            let updates = self
                .estimator
                .update_barometer_biases(sample.ts.as_micros(), measurements)?;
            for (anchored, chain) in self.bias_anchored.iter_mut().zip(&updates) {
                *anchored |= chain.iter().flatten().any(|update| update.accepted);
            }
            let selected = &updates[self.estimator.selected_imu().index()];
            if selected.iter().flatten().any(|update| update.accepted) {
                self.anchored_at.get_or_insert(sample.ts);
                self.gnss_height_std_m = height_std_m;
            }
        } else if self.anchored_at.is_none() {
            return Ok(());
        }
        self.track_gnss_velocity(sample.ts, -sample.pvt.vel_down);
        self.last_gnss_fusion = Some((sample.src, sample.ts));
        Ok(())
    }

    /// Bias observations for barometers sampled close to the GNSS epoch.
    /// A settled bias is not refitted while pressure is stable; otherwise a
    /// slowly wandering indoor GNSS fix would move height through the bias.
    fn barometer_bias_measurements(
        &self,
        ts: Instant,
        gnss_height_m: f32,
        std_m: f32,
    ) -> [Option<BarometerBiasMeasurement>; BAROMETER_COUNT] {
        let bias_variance = ImuId::ALL.map(|id| {
            self.estimator
                .uncertainty(sef_imu(id.index()))
                .barometer_bias_variance_m2
        });
        core::array::from_fn(|index| {
            let (baro_ts, pressure_altitude_m) = self.last_baro_height[index]?;
            let close =
                ts.as_micros().abs_diff(baro_ts.as_micros()) <= BARO_FOR_GNSS_MAX_AGE.as_micros();
            let needs_fit = self.anchored_at.is_none()
                || self.barometer_bias_tracker.should_fit(
                    index,
                    bias_variance[0][index].max(bias_variance[1][index]),
                    BARO_HEIGHT_STD_M,
                );
            (close && needs_fit).then_some(BarometerBiasMeasurement {
                bias_m: pressure_altitude_m - gnss_height_m,
                std_m,
            })
        })
    }

    /// An unanchored startup can leave the vertical state too far from MSL
    /// for the innovation gate, so the first GNSS fit starts a fresh filter.
    /// The latest barometer heights are kept: the first fit anchors their
    /// biases, and the next pressure samples then set MSL height.
    fn restart_for_first_anchor(&mut self) -> Result<(), EstimatorError> {
        self.estimator = new_estimator(GYRO_RANGE_DPS)?;
        self.bias_anchored = [false; IMU_COUNT];
        self.pressure_after_anchor = [false; IMU_COUNT];
        self.last_gnss_velocity = None;
        self.gnss_displacement_m = 0.0;
        self.barometer_bias_tracker = BarometerBiasTracker::default();
        Ok(())
    }

    fn track_gnss_velocity(&mut self, ts: Instant, velocity_mps: f32) {
        if let Some((last, previous_velocity_mps)) = self.last_gnss_velocity {
            let elapsed = ts.saturating_duration_since(last);
            if elapsed <= GNSS_VELOCITY_FRESH {
                self.gnss_displacement_m += 0.5
                    * (previous_velocity_mps + velocity_mps)
                    * (elapsed.as_micros() as f32 / 1_000_000.0);
            } else {
                self.barometer_bias_tracker.reset_windows();
            }
        }
        self.last_gnss_velocity = Some((ts, velocity_mps));
    }

    fn msl_ready(&self, index: usize) -> bool {
        self.bias_anchored[index] && self.pressure_after_anchor[index]
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

        let selected = imu.index();
        let state = self.estimator.selected_state();
        let uncertainty = self.estimator.selected_uncertainty();
        signals::STATE_ESTIMATE_WATCH.sender().send(StateEstimate {
            ts,
            msl_ready: self.msl_ready(selected),
            height_msl_m: state.height_m,
            velocity_mps: state.velocity_mps,
            height_std_m: libm::sqrtf(uncertainty.height_variance_m2).max(self.gnss_height_std_m),
            velocity_std_mps: libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
            orientation_body_to_ned_wxyz: self.estimator.orientation_body_to_ned_wxyz(imu),
            selected_imu: ImuId::ALL[selected],
            redundancy_ready: self.estimator.redundancy_ready(),
        });

        let scores = self.estimator.consistency_scores();
        for id in ImuId::ALL {
            let index = id.index();
            let imu = sef_imu(index);
            let state = self.estimator.state(imu);
            let uncertainty = self.estimator.uncertainty(imu);
            signals::submit_sd_log(SdLogRecord::State(SefLogSample {
                ts,
                imu: id,
                selected: index == selected,
                msl_ready: self.msl_ready(index),
                redundancy_ready: self.estimator.redundancy_ready(),
                selected_gnss: self.gnss.selected(),
                height_msl_m: state.height_m,
                velocity_mps: state.velocity_mps,
                barometer_bias_m: state.barometer_bias_m,
                height_std_m: libm::sqrtf(uncertainty.height_variance_m2),
                velocity_std_mps: libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
                barometer_bias_std_m: uncertainty.barometer_bias_variance_m2.map(libm::sqrtf),
                consistency_score: scores[index],
                orientation_body_to_ned_wxyz: self.estimator.orientation_body_to_ned_wxyz(imu),
            }));
        }
        self.last_output = Some(now);
    }
}

fn sef_imu(index: usize) -> asteria_sef_light::ImuId {
    asteria_sef_light::ImuId::from_index(index).expect("firmware IMU ID must map to SEF-light")
}

mod gnss_selection;
mod stream;
pub use stream::task;
