//! State estimation from the calibrated sensor streams.
//!
//! Readings go through a [`BufferedTimeHorizon`], which hands them to SEF-light
//! in time order: IMU samples arrive in FIFO batches up to ~15 ms old, and a
//! late sample would make SEF-light replay its history.
//! Every reading publishes the estimator's
//! [`NavigationState`](asteria_sef_core::NavigationState) to
//! [`signals::STATE_ESTIMATE_WATCH`]; every [`LOG_PERIOD`] each SEF-light chain
//! is also logged to SD.
//!
//! Positions are north-east-down metres from [`launch_site`], whose down is
//! the negated MSL height. GNSS fixes are converted into that frame before
//! fusion.

use asteria_sef_core::{
    BarometerAided, BarometerInput, BufferedTimeHorizon, DualGnssSelector, GeodeticPosition,
    GeodeticReference, GnssAided, GnssInput, GnssSelectorConfig, GnssSolution, ImuAided, ImuInput,
    MagnetometerAided, MagnetometerInput, SelectorConfig, StateEstimator, TimeHorizonConfig,
    UpdateError,
};
use asteria_sef_light::{
    DualVerticalEstimator, EstimatorError, ImuAttitudeConfig, STANDARD_GRAVITY_MPS2,
    VerticalEstimatorSelectorConfig, VerticalFilterConfig,
};
use defmt::{Debug2Format, warn};
use embassy_futures::select::{Either4, select4};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant};

use crate::calibration;
use crate::sensors::{GNSS_COUNT, IMU_COUNT, ImuId};
use crate::signals;
use crate::tasks::readout::imu::GYRO_RANGE_DPS;
use crate::tasks::readout::mag;
use crate::types::{BaroSample, GnssSample, ImuSample, MagSample, SefLogSample};

// Origin of the estimator's north-east-down frame. It only has to be within a
// few hundred metres of the flight.
const LAUNCH_SITE_LATITUDE_DEG: f64 = 47.405_336;
const LAUNCH_SITE_LONGITUDE_DEG: f64 = 8.631_050;

/// The launch site at MSL height zero, so down is the negated MSL height.
pub fn launch_site() -> GeodeticReference {
    GeodeticReference::new(LAUNCH_SITE_LATITUDE_DEG, LAUNCH_SITE_LONGITUDE_DEG, 0.0)
        .expect("launch site must be a valid geodetic reference")
}

// With barometers only, the height standard deviation settles near 140 m, the
// barometer bias prior. The first GNSS height update brings it under 30 m.
const MSL_REFERENCED_HEIGHT_STD_M: f32 = 100.0;

/// Whether a height this uncertain has been referenced to MSL by GNSS, not just
/// carried by the barometers.
pub fn height_msl_referenced(height_std_m: f32) -> bool {
    height_std_m < MSL_REFERENCED_HEIGHT_STD_M
}

type Estimator = BufferedTimeHorizon<DualVerticalEstimator<HISTORY_CAPACITY>, PENDING_CAPACITY>;

const HOLDBACK_US: u64 = 35_000;
// Two IMUs at 833 Hz fill about 60 slots during the holdback.
const PENDING_CAPACITY: usize = 128;
// Rows per second of the per-chain state in the SD log.
const LOG_PERIOD: Duration = Duration::from_millis(50);
const IMU_FRESH: Duration = Duration::from_millis(100);
const WARNING_PERIOD: Duration = Duration::from_secs(1);
// Two 833 Hz IMUs produce about 667 events in 400 ms. The remaining capacity
// covers barometers, GNSS, and interrupt scheduling jitter.
const HISTORY_CAPACITY: usize = 768;
const MAX_AIDING_DELAY_US: u64 = 400_000;
const BARO_HEIGHT_STD_M: f32 = 1.5;
// A magnetometer sample keeps aiding attitude for two and a half sample
// periods, so one late or rejected sample does not drop magnetometer aiding.
const MAG_MAX_AGE_US: u64 = mag::SAMPLE_INTERVAL.as_micros() * 5 / 2;
// Smallest GNSS standard deviation passed on, for receivers reporting zero.
const GNSS_MIN_STD: f32 = 0.1;
// GNSS height errors persist for a minute or more, so 20 Hz epochs are not
// independent: at rest the height spread 9 m while the receiver reported 3 m.
const GNSS_HEIGHT_STD_SCALE: f32 = 10.0;
// Walking with the board, the receiver reported up to 0.75 m/s of vertical
// speed the barometers did not see, which bent the height by up to 1 m.
const GNSS_SPEED_STD_SCALE: f32 = 10.0;
// Read only by estimators that model them, not by SEF-light: the launch site's
// magnetic field direction (declination about 3°, inclination about 64°), the
// noise of its normalized components, and the pressure counterpart of
// BARO_HEIGHT_STD_M at about 0.12 hPa per metre.
const LOCAL_MAGNETIC_FIELD_NED_NT: [f32; 3] = [21_300.0, 1_200.0, 43_000.0];
const MAG_DIRECTION_STD: f32 = 0.05;
const BARO_REFERENCE_PRESSURE_HPA: f32 = 1_013.25;
const BARO_PRESSURE_STD_HPA: f32 = 0.18;

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
    gnss: DualGnssSelector,
    reference: GeodeticReference,
    last_log: Option<Instant>,
    last_warning: Option<Instant>,
}

impl Processor {
    fn new() -> Result<Self, EstimatorError> {
        let filter = VerticalFilterConfig::new(
            10.0,       // healthy acceleration noise, m/s² per sample
            20.0,       // degraded acceleration noise, m/s² per sample
            [0.02; 2],  // barometer-bias random walk, m/√s
            1_000.0,    // initial height uncertainty, m; GNSS references it to MSL
            3.0,        // initial vertical-velocity uncertainty, m/s
            [200.0; 2], // initial pressure-altitude bias uncertainty, m
            5.0,        // measurement innovation gate, standard deviations
        )?;
        let attitude = ImuAttitudeConfig::new(
            2.0, // AHRS feedback gain
            GYRO_RANGE_DPS,
            10.0, // accelerometer rejection angle, degrees
            300,  // rejected samples before acceleration recovery
        )?
        .with_magnetometer(
            20.0, // magnetic rejection angle, degrees
            MAG_MAX_AGE_US,
        )?;
        let selection = SelectorConfig::new(
            0.0025,    // IMU score improvement required for a handover
            5_000_000, // required improvement duration and minimum time between handovers, µs
        )
        .ok_or(EstimatorError::OutOfRangeInput)?;
        let selector = VerticalEstimatorSelectorConfig::new(
            0.95,    // previous score weight
            25.0,    // maximum contribution from one innovation
            10.0,    // degraded acceleration penalty
            100_000, // maximum IMU sample age, µs
            selection,
        )?;
        let gnss = GnssSelectorConfig::new(
            3,       // minimum fix tier
            500_000, // minimum time between receiver handovers, µs
        );
        Ok(Self {
            estimator: BufferedTimeHorizon::new(
                DualVerticalEstimator::new(
                    filter,
                    [attitude; IMU_COUNT],
                    selector,
                    MAX_AIDING_DELAY_US,
                )?,
                TimeHorizonConfig {
                    max_span_us: HOLDBACK_US,
                },
            ),
            gnss: DualGnssSelector::new(gnss),
            reference: launch_site(),
            last_log: None,
            last_warning: None,
        })
    }

    fn update_imu(&mut self, sample: ImuSample) -> Result<(), UpdateError> {
        const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        self.estimator.update_imu(ImuInput {
            timestamp_us: sample.ts.as_micros(),
            sensor_index: sample.src.index(),
            acceleration_body_mps2: [sample.accel.x, sample.accel.y, sample.accel.z]
                .map(|g| g * STANDARD_GRAVITY_MPS2),
            angular_rate_body_rad_s: [sample.gyro.x, sample.gyro.y, sample.gyro.z]
                .map(|dps| dps * DEG_TO_RAD),
        })
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
        self.estimator.update_magnetometer(MagnetometerInput {
            timestamp_us: sample.ts.as_micros(),
            sensor_index: sample.src.index(),
            field_body: field,
            reference_field_ned: LOCAL_MAGNETIC_FIELD_NED_NT,
            direction_noise_std: MAG_DIRECTION_STD,
        })
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

    /// Fuses one receiver's solution on its own, weighted by its reported
    /// accuracy, if the receiver selector picks it.
    fn update_gnss(&mut self, sample: GnssSample) -> Result<(), UpdateError> {
        let mut candidates = [None; GNSS_COUNT];
        candidates[sample.src.index()] = Some(asteria_sef_core::GnssSample {
            measurement: sample,
            // 3 is a usable 3D fix; the selector ignores tiers below.
            fix_tier: match sample.pvt.fix_type {
                _ if !sample.pvt.fix_ok => 0,
                ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning => 3,
                ublox::GpsFix::Fix2D => 2,
                _ => 0,
            },
            pdop_centi: sample.pvt.pdop_centi,
        });
        let Some(selected) = self.gnss.select(sample.ts.as_micros(), candidates) else {
            return Ok(());
        };
        let fix = selected.measurement;
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
        let state = self.estimator.navigation_state();
        let ts = Instant::from_micros(state.time_us);
        if now.saturating_duration_since(ts) > IMU_FRESH {
            return;
        }
        signals::STATE_ESTIMATE_WATCH.sender().send(state);

        if self
            .last_log
            .is_some_and(|last| now.saturating_duration_since(last) < LOG_PERIOD)
        {
            return;
        }
        let sef = self.estimator.inner();
        let selected = sef.selected_imu().index();
        let scores = sef.consistency_scores();
        for (id, imu) in ImuId::ALL.into_iter().zip(asteria_sef_light::ImuId::ALL) {
            let state = sef.state(imu);
            let uncertainty = sef.uncertainty(imu);
            let height_std_m = libm::sqrtf(uncertainty.height_variance_m2);
            signals::submit_state(SefLogSample {
                ts,
                imu: id,
                selected: imu.index() == selected,
                msl_ready: height_msl_referenced(height_std_m),
                redundancy_ready: sef.redundancy_ready(),
                height_msl_m: state.height_m,
                velocity_mps: state.velocity_mps,
                barometer_bias_m: state.barometer_bias_m,
                height_std_m,
                velocity_std_mps: libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
                barometer_bias_std_m: uncertainty.barometer_bias_variance_m2.map(libm::sqrtf),
                consistency_score: scores[imu.index()],
                orientation_body_to_ned_wxyz: sef.orientation_body_to_ned_wxyz(imu),
            });
        }
        self.last_log = Some(now);
    }
}

impl GnssSolution for GnssSample {
    fn is_valid(&self) -> bool {
        let pvt = &self.pvt;
        pvt.latitude_deg.is_finite()
            && pvt.longitude_deg.is_finite()
            && [
                pvt.height_msl_m,
                pvt.velocity_north_mps,
                pvt.velocity_east_mps,
                pvt.velocity_down_mps,
                pvt.speed_accuracy_mps,
            ]
            .iter()
            .all(|value| value.is_finite())
    }
}
