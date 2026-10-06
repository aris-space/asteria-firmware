//! Feeds the calibrated sensor streams into SEF-light and publishes its estimate.
//!
//! SEF-light fuses every reading [`FUSION_DELAY_US`] behind the newest IMU
//! sample, in time order, and carries the state forward to it with the IMU.
//! Positions are NED metres from [`launch_site`].

use asteria_sef_core::{
    BarometerAided, BarometerInput, GeodeticPosition, GeodeticReference, GnssAided, GnssInput,
    ImuAided, ImuInput, MagnetometerAided, MagnetometerInput, SelectorConfig, StateEstimator,
    UpdateError,
};
use asteria_sef_light::{
    DualVerticalEstimator, EstimatorError, ImuAttitudeConfig, STANDARD_GRAVITY_MPS2,
    StationaryConfig, VerticalEstimatorSelectorConfig, VerticalFilterConfig,
};
use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either4, select4};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant};

use crate::calibration;
use crate::sensors::{IMU_COUNT, ImuId};
use crate::signals;
use crate::tasks::readout::imu::{ACCEL_RANGE_G, GYRO_SATURATION_DPS};
use crate::tasks::readout::mag;
use crate::types::{BaroSample, GnssSample, ImuSample, MagSample, SefLogSample};

// Only has to be within a few hundred metres of the flight.
const LAUNCH_SITE_LATITUDE_DEG: f64 = 47.405_336;
const LAUNCH_SITE_LONGITUDE_DEG: f64 = 8.631_050;

// GNSS fixes reach the estimator up to ~145 ms after their time of validity.
const FUSION_DELAY_US: u64 = 200_000;
const PENDING_CAPACITY: usize = 512; // ~350 readings per fusion delay
const PUBLISH_PERIOD: Duration = Duration::from_millis(50);
const IMU_FRESH: Duration = Duration::from_millis(100);
const WARNING_PERIOD: Duration = Duration::from_secs(1);

// Accelerations are m/s² per sample. The IMU noise is far above the sensor's own,
// mostly for attitude errors, and the barometer's is 5x its measured 0.3 m to match.
const ACCELERATION_NOISE: f32 = 10.0;
const DEGRADED_ACCELERATION_NOISE: f32 = 20.0;
const SATURATED_ACCELERATION_NOISE: f32 = 1_000.0; // a clipped sample hides how much
const FREE_FALL_ACCELERATION_NOISE: f32 = 1.0; // falling, up is -g whatever the attitude
const BARO_HEIGHT_STD_M: f32 = 1.5;
const BARO_BIAS_WALK_M_PER_SQRT_S: f32 = 0.02;
const INITIAL_HEIGHT_STD_M: f32 = 1_000.0; // until GNSS references it to MSL
const INITIAL_VELOCITY_STD_MPS: f32 = 3.0;
const INITIAL_BARO_BIAS_STD_M: f32 = 200.0;
const INNOVATION_GATE_SIGMA: f32 = 5.0;

// GNSS errors last a minute or more, so 20 Hz epochs aren't independent: at rest
// the height spread 9 m against a reported 3 m, and walking it reported vertical
// speed the barometers didn't see.
const GNSS_HEIGHT_STD_SCALE: f32 = 10.0;
const GNSS_SPEED_STD_SCALE: f32 = 10.0;
const GNSS_MIN_STD: f32 = 0.1; // receivers can report zero

const AHRS_GAIN: f32 = 2.0;
const ACCEL_REJECTION_DEG: f32 = 10.0;
const ACCEL_RECOVERY_SAMPLES: u32 = 300;
const MAG_REJECTION_DEG: f32 = 20.0;
// One late or rejected sample shouldn't drop magnetometer aiding.
const MAG_MAX_AGE_US: u64 = mag::SAMPLE_INTERVAL.as_micros() * 5 / 2;
// The AHRS resets its attitude past this, so only when the gyro really clips.
const GYRO_RESET_DPS: f32 = 0.99 * GYRO_SATURATION_DPS;
// Calibration scales a clipped reading by a few percent either way.
const ACCEL_SATURATION_MPS2: f32 = 0.97 * ACCEL_RANGE_G * STANDARD_GRAVITY_MPS2;

// Only for estimators that model them, not SEF-light.
const LOCAL_MAGNETIC_FIELD_NED_NT: [f32; 3] = [21_300.0, 1_200.0, 43_000.0];
const MAG_DIRECTION_STD: f32 = 0.05;
const BARO_REFERENCE_PRESSURE_HPA: f32 = 1_013.25;
const BARO_PRESSURE_STD_HPA: f32 = 0.18; // BARO_HEIGHT_STD_M at ~0.12 hPa/m

// Barometers alone settle near 140 m (their bias prior); the first GNSS height
// brings it under 30 m.
const MSL_REFERENCED_HEIGHT_STD_M: f32 = 100.0;

/// The launch site at MSL height zero, so down is the negated MSL height.
pub fn launch_site() -> GeodeticReference {
    GeodeticReference::new(LAUNCH_SITE_LATITUDE_DEG, LAUNCH_SITE_LONGITUDE_DEG, 0.0)
        .expect("launch site must be a valid geodetic reference")
}

/// Whether GNSS has referenced the height to MSL, rather than only the barometers.
pub fn height_msl_referenced(height_std_m: f32) -> bool {
    height_std_m < MSL_REFERENCED_HEIGHT_STD_M
}

type Estimator = DualVerticalEstimator<PENDING_CAPACITY>;

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut imu = signals::IMU_CHANNEL
        .subscriber()
        .expect("SEF: subscriber slot");
    let mut mag = signals::MAG_CHANNEL
        .subscriber()
        .expect("SEF: subscriber slot");
    let mut gnss = signals::GNSS_CHANNEL
        .subscriber()
        .expect("SEF: subscriber slot");
    let mut baro = signals::BARO_CHANNEL
        .subscriber()
        .expect("SEF: subscriber slot");
    let mut processor = Processor::new().expect("SEF-light configuration must be valid");
    loop {
        let result = match select4(
            imu.next_message(),
            mag.next_message(),
            gnss.next_message(),
            baro.next_message(),
        )
        .await
        {
            Either4::First(m) => {
                received(m, "IMU").map(|reading| processor.update_imu(reading.cal))
            }
            Either4::Second(m) => {
                received(m, "mag").map(|reading| processor.update_mag(reading.cal))
            }
            Either4::Third(m) => {
                received(m, "GNSS").map(|reading| processor.update_gnss(reading.cal))
            }
            Either4::Fourth(m) => {
                received(m, "baro").map(|reading| processor.update_baro(reading.cal))
            }
        };
        if let Some(result) = result {
            processor.report(result);
            processor.publish();
        }
    }
}

fn received<T>(message: WaitResult<T>, kind: &str) -> Option<T> {
    match message {
        WaitResult::Message(reading) => Some(reading),
        WaitResult::Lagged(lost) => {
            warn!("SEF: fell behind and lost {} {} readings", lost, kind);
            None
        }
    }
}

struct Processor {
    estimator: Estimator,
    reference: GeodeticReference,
    last_publish: Option<Instant>,
    last_warning: Option<Instant>,
    msl_referenced: bool,
    selected_imu: usize,
}

impl Processor {
    fn new() -> Result<Self, EstimatorError> {
        let filter = VerticalFilterConfig::new(
            ACCELERATION_NOISE,
            DEGRADED_ACCELERATION_NOISE,
            [BARO_BIAS_WALK_M_PER_SQRT_S; 2],
            INITIAL_HEIGHT_STD_M,
            INITIAL_VELOCITY_STD_MPS,
            [INITIAL_BARO_BIAS_STD_M; 2],
            INNOVATION_GATE_SIGMA,
        )?
        .with_saturated_acceleration_noise(SATURATED_ACCELERATION_NOISE)?
        .with_free_fall_acceleration_noise(FREE_FALL_ACCELERATION_NOISE)?;
        let attitude = ImuAttitudeConfig::new(
            AHRS_GAIN,
            GYRO_RESET_DPS,
            ACCEL_REJECTION_DEG,
            ACCEL_RECOVERY_SAMPLES,
        )?
        .with_magnetometer(MAG_REJECTION_DEG, MAG_MAX_AGE_US)?
        .with_accelerometer_saturation(ACCEL_SATURATION_MPS2)?;
        let selector = VerticalEstimatorSelectorConfig {
            score_memory: 0.95,
            maximum_nis_contribution: 25.0,
            degraded_score_penalty: 10.0,
            imu_selection: SelectorConfig {
                switch_hysteresis: 0.0025,
                minimum_dwell_time_us: 5_000_000,
                maximum_age_us: 100_000,
            },
            gnss_selection: SelectorConfig {
                minimum_dwell_time_us: 500_000,
                maximum_age_us: 250_000,
                ..SelectorConfig::default()
            },
        };
        let stationary = StationaryConfig {
            maximum_angular_rate_rad_s: 0.05,
            maximum_specific_force_error_mps2: 0.3,
            minimum_duration_us: 500_000,
            update_interval_us: 50_000,
            velocity_std_mps: 0.05,
        };
        Ok(Self {
            estimator: DualVerticalEstimator::new(
                filter,
                [attitude; IMU_COUNT],
                selector,
                stationary,
                FUSION_DELAY_US,
            )?,
            reference: launch_site(),
            last_publish: None,
            last_warning: None,
            msl_referenced: false,
            selected_imu: 0,
        })
    }

    fn update_imu(&mut self, sample: ImuSample) -> Result<(), UpdateError> {
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        ImuAided::update_imu(
            &mut self.estimator,
            ImuInput {
                timestamp_us: sample.ts.as_micros(),
                sensor_index: sample.src.index(),
                acceleration_body_mps2: [sample.accel.x, sample.accel.y, sample.accel.z]
                    .map(|g| g * STANDARD_GRAVITY_MPS2),
                angular_rate_body_rad_s: [sample.gyro.x, sample.gyro.y, sample.gyro.z]
                    .map(|dps| dps * DEG_TO_RAD),
            },
        )
    }

    fn update_mag(&mut self, sample: MagSample) -> Result<(), UpdateError> {
        let field = [sample.x, sample.y, sample.z];
        let field_nt = libm::sqrtf(field.iter().map(|value| value * value).sum());
        if !calibration::mag::CAL
            .applied(sample.src)
            .correction
            .accepts_field(field_nt)
        {
            return Ok(());
        }
        // Each magnetometer aids the attitude chain of the IMU with the same index.
        MagnetometerAided::update_magnetometer(
            &mut self.estimator,
            MagnetometerInput {
                timestamp_us: sample.ts.as_micros(),
                sensor_index: sample.src.index(),
                field_body: field,
                reference_field_ned: LOCAL_MAGNETIC_FIELD_NED_NT,
                direction_noise_std: MAG_DIRECTION_STD,
            },
        )
    }

    fn update_baro(&mut self, sample: BaroSample) -> Result<(), UpdateError> {
        self.estimator.update_barometer(BarometerInput {
            timestamp_us: sample.ts.as_micros(),
            sensor_index: sample.src.index(),
            height_m: sample.pressure_altitude_m(),
            height_std_m: BARO_HEIGHT_STD_M,
            pressure_hpa: sample.pressure_mbar,
            reference_pressure_hpa: BARO_REFERENCE_PRESSURE_HPA,
            pressure_std_hpa: BARO_PRESSURE_STD_HPA,
        })
    }

    fn report(&mut self, result: Result<(), UpdateError>) {
        if let Err(error) = result {
            let now = Instant::now();
            if self
                .last_warning
                .is_none_or(|last| now.saturating_duration_since(last) >= WARNING_PERIOD)
            {
                warn!("SEF: update failed: {:?}", Debug2Format(&error));
                self.last_warning = Some(now);
            }
        }
    }

    /// Every 3D fix goes in; SEF-light picks the receiver.
    fn update_gnss(&mut self, fix: GnssSample) -> Result<(), UpdateError> {
        if !fix.pvt.has_3d_fix() {
            return Ok(());
        }
        let position_ned_m = self.reference.project(GeodeticPosition {
            latitude_deg: fix.pvt.latitude_deg,
            longitude_deg: fix.pvt.longitude_deg,
            height_msl_m: fix.pvt.height_msl_m,
        });
        let horizontal_std_m = (fix.pvt.horizontal_accuracy_mm as f32 / 1_000.0).max(GNSS_MIN_STD);
        let height_std_m = (fix.pvt.vertical_accuracy_mm as f32 / 1_000.0 * GNSS_HEIGHT_STD_SCALE)
            .max(GNSS_MIN_STD);
        let speed_std_mps = fix.pvt.speed_accuracy_mps.max(GNSS_MIN_STD);
        let vertical_speed_std_mps =
            (fix.pvt.speed_accuracy_mps * GNSS_SPEED_STD_SCALE).max(GNSS_MIN_STD);
        self.estimator.update_gnss(GnssInput {
            timestamp_us: fix.ts.as_micros(),
            sensor_index: fix.src.index(),
            height_m: -position_ned_m[2],
            vertical_velocity_mps: -fix.pvt.velocity_down_mps,
            height_std_m,
            vertical_velocity_std_mps: vertical_speed_std_mps,
            position_ned_m,
            velocity_ned_mps: [
                fix.pvt.velocity_north_mps,
                fix.pvt.velocity_east_mps,
                fix.pvt.velocity_down_mps,
            ],
            position_std_m: [horizontal_std_m, horizontal_std_m, height_std_m],
            velocity_std_mps: [speed_std_mps, speed_std_mps, vertical_speed_std_mps],
        })
    }

    fn publish(&mut self) {
        let now = Instant::now();
        if self
            .last_publish
            .is_some_and(|last| now.saturating_duration_since(last) < PUBLISH_PERIOD)
        {
            return;
        }
        self.last_publish = Some(now);
        let state = self.estimator.navigation_state();
        if now.saturating_duration_since(Instant::from_micros(state.time_us)) > IMU_FRESH {
            return;
        }
        signals::STATE_ESTIMATE_WATCH.sender().send(state);

        let msl_referenced = height_msl_referenced(state.position_std_ned_m[2]);
        if msl_referenced != self.msl_referenced {
            if msl_referenced {
                info!("SEF: height referenced to MSL");
            } else {
                warn!("SEF: height no longer referenced to MSL");
            }
            self.msl_referenced = msl_referenced;
        }
        let sef = &self.estimator;
        let selected = sef.selected_imu();
        if selected != self.selected_imu {
            info!("SEF: switched to {}", ImuId::ALL[selected]);
            self.selected_imu = selected;
        }
        let ts = Instant::from_micros(state.time_us);
        let scores = sef.consistency_scores();
        for id in ImuId::ALL {
            let imu = id.index();
            let state = sef.state(imu);
            let uncertainty = sef.uncertainty(imu);
            let height_std_m = libm::sqrtf(uncertainty.height_variance_m2);
            signals::submit_state(SefLogSample {
                ts,
                imu: id,
                selected: imu == selected,
                msl_ready: height_msl_referenced(height_std_m),
                redundancy_ready: sef.redundancy_ready(),
                height_msl_m: state.height_m,
                velocity_mps: state.velocity_mps,
                barometer_bias_m: state.barometer_bias_m,
                height_std_m,
                velocity_std_mps: libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
                barometer_bias_std_m: uncertainty.barometer_bias_variance_m2.map(libm::sqrtf),
                consistency_score: scores[imu],
                orientation_body_to_ned_wxyz: sef.orientation_body_to_ned_wxyz(imu),
            });
        }
    }
}
