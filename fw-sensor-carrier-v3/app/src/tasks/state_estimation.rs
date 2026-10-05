//! SEF-light vertical estimation from the calibrated sensor streams.
//!
//! Samples are held for [`HOLDBACK`] and handed to the estimator in timestamp
//! order. SEF-light accepts late samples, but each one replays its history, and
//! IMU samples arrive in FIFO batches that are already up to ~15 ms old.
//! Each output tick publishes the selected chain to
//! [`signals::STATE_ESTIMATE_WATCH`] and logs every chain to SD.

use asteria_sef_light::{
    BarometerId, DualVerticalEstimator, EstimatorError, GnssSelectorConfig, ImuAttitudeConfig,
    ImuMeasurement, PressureMeasurement, STANDARD_GRAVITY_MPS2, SelectorConfig,
    VerticalEstimatorSelectorConfig, VerticalFilterConfig, VerticalGnssMeasurement,
};
use defmt::{Debug2Format, warn};
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_time::{Duration, Instant, Timer};
use heapless::Vec;

use crate::calibration;
use crate::sensors::{GNSS_COUNT, IMU_COUNT, ImuId};
use crate::signals;
use crate::tasks::readout::imu::GYRO_RANGE_DPS;
use crate::types::{BaroSample, GnssSample, ImuSample, MagSample, SefLogSample, StateEstimate};

const HOLDBACK: Duration = Duration::from_millis(35);
// Two IMUs at 833 Hz fill about 60 slots during the holdback.
const PENDING_CAPACITY: usize = 128;
const OUTPUT_PERIOD: Duration = Duration::from_millis(50);
const IMU_FRESH: Duration = Duration::from_millis(100);
const WARNING_PERIOD: Duration = Duration::from_secs(1);
// Two 833 Hz IMUs produce about 667 events in 400 ms. The remaining capacity
// covers barometers, GNSS, and interrupt scheduling jitter.
const HISTORY_CAPACITY: usize = 768;
const MAX_AIDING_DELAY_US: u64 = 400_000;
const BARO_HEIGHT_STD_M: f32 = 1.5;
// Smallest GNSS standard deviation passed on, for receivers reporting zero.
const GNSS_MIN_STD: f32 = 0.1;

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
    let mut pending = Vec::<Event, PENDING_CAPACITY>::new();
    loop {
        let oldest = pending
            .iter()
            .enumerate()
            .min_by_key(|(_, event)| event.ts())
            .map(|(index, event)| (index, event.ts() + HOLDBACK));
        let due = oldest.map_or(Instant::MAX, |(_, due)| due);
        let next = select4(
            imu.next_message_pure(),
            mag.next_message_pure(),
            gnss.next_message_pure(),
            baro.next_message_pure(),
        );
        let event = match select(Timer::at(due), next).await {
            Either::First(()) => {
                let (index, _) = oldest.expect("a sample is due");
                processor.handle(pending.swap_remove(index));
                continue;
            }
            Either::Second(Either4::First(reading)) => Event::Imu(reading.cal),
            Either::Second(Either4::Second(reading)) => Event::Mag(reading.cal),
            Either::Second(Either4::Third(reading)) => Event::Gnss(reading.cal),
            Either::Second(Either4::Fourth(reading)) => Event::Baro(reading.cal),
        };
        if pending.push(event).is_err() {
            warn!("SEF: input buffer full, dropped a sample");
        }
    }
}

#[derive(Clone, Copy)]
enum Event {
    Imu(ImuSample),
    Mag(MagSample),
    Gnss(GnssSample),
    Baro(BaroSample),
}

impl Event {
    fn ts(self) -> Instant {
        match self {
            Self::Imu(sample) => sample.ts,
            Self::Mag(sample) => sample.ts,
            Self::Gnss(sample) => sample.ts,
            Self::Baro(sample) => sample.ts,
        }
    }
}

/// Both receivers' solutions for one GPS epoch. SEF-light selects or blends
/// the receivers per epoch, so it must see them together.
#[derive(Clone, Copy)]
struct GnssEpoch {
    itow_ms: u32,
    ts: Instant,
    samples: [Option<GnssSample>; GNSS_COUNT],
}

struct Processor {
    estimator: DualVerticalEstimator<HISTORY_CAPACITY>,
    gnss_epoch: Option<GnssEpoch>,
    /// Per chain: a GNSS height update was accepted, so its height is MSL.
    msl_ready: [bool; IMU_COUNT],
    last_output: Option<Instant>,
    last_warning: Option<Instant>,
}

impl Processor {
    fn new() -> Result<Self, EstimatorError> {
        let filter = VerticalFilterConfig::new(
            10.0,       // healthy acceleration noise, m/s² per sample
            20.0,       // degraded acceleration noise, m/s² per sample
            [0.02; 2],  // barometer-bias random walk, m/√s
            1_000.0,    // initial height uncertainty, m; GNSS establishes MSL
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
        .with_magnetic_rejection(20.0)?;
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
            4.0,     // inter-receiver consistency gate, standard deviations
            500_000, // minimum time between receiver handovers, µs
        )
        .ok_or(EstimatorError::OutOfRangeInput)?;
        Ok(Self {
            estimator: DualVerticalEstimator::new(
                filter,
                [attitude; IMU_COUNT],
                selector,
                gnss,
                MAX_AIDING_DELAY_US,
            )?,
            gnss_epoch: None,
            msl_ready: [false; IMU_COUNT],
            last_output: None,
            last_warning: None,
        })
    }

    fn handle(&mut self, event: Event) {
        let result = match event {
            Event::Imu(sample) => {
                const DEG_TO_RAD: f32 = core::f32::consts::PI / 180.0;
                let measurement = ImuMeasurement {
                    acceleration_body_mps2: [sample.accel.x, sample.accel.y, sample.accel.z]
                        .map(|g| g * STANDARD_GRAVITY_MPS2),
                    angular_rate_body_rad_s: [sample.gyro.x, sample.gyro.y, sample.gyro.z]
                        .map(|dps| dps * DEG_TO_RAD),
                };
                self.estimator
                    .update_imu(
                        asteria_sef_light::ImuId::ALL[sample.src.index()],
                        sample.ts.as_micros(),
                        measurement,
                    )
                    .map(|_| ())
            }
            Event::Mag(sample) => {
                let field = [sample.x, sample.y, sample.z];
                let field_nt = libm::sqrtf(field.iter().map(|value| value * value).sum());
                let calibration = calibration::mag::CAL.applied(sample.src);
                if calibration.correction.accepts_field(field_nt) {
                    // Each magnetometer aids the attitude chain of the IMU with the same index.
                    self.estimator.update_magnetometer(
                        asteria_sef_light::ImuId::ALL[sample.src.index()],
                        sample.ts.as_micros(),
                        field,
                    )
                } else {
                    Ok(())
                }
            }
            Event::Gnss(sample) => self.update_gnss(sample),
            Event::Baro(sample) => self
                .estimator
                .update_pressure(
                    sample.ts.as_micros(),
                    BarometerId::ALL[sample.src.index()],
                    PressureMeasurement {
                        height_m: sample.pressure_altitude_m(),
                        height_std_m: BARO_HEIGHT_STD_M,
                    },
                )
                .map(|_| ()),
        };
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
        self.publish();
    }

    /// Collects each receiver's solution into its GPS epoch. An epoch is fused
    /// once both receivers reported it, or when the next epoch starts.
    fn update_gnss(&mut self, sample: GnssSample) -> Result<(), EstimatorError> {
        if let Some(epoch) = self.gnss_epoch
            && epoch.itow_ms != sample.pvt.itow_ms
        {
            self.gnss_epoch = None;
            self.fuse_gnss(epoch)?;
        }
        let epoch = self.gnss_epoch.get_or_insert(GnssEpoch {
            itow_ms: sample.pvt.itow_ms,
            ts: sample.ts,
            samples: [None; GNSS_COUNT],
        });
        epoch.samples[sample.src.index()] = Some(sample);
        if epoch.samples.iter().all(Option::is_some) {
            let epoch = *epoch;
            self.gnss_epoch = None;
            self.fuse_gnss(epoch)?;
        }
        Ok(())
    }

    fn fuse_gnss(&mut self, epoch: GnssEpoch) -> Result<(), EstimatorError> {
        let samples = epoch.samples.map(|sample| {
            sample.map(|sample| asteria_sef_light::GnssSample {
                measurement: VerticalGnssMeasurement {
                    height_m: sample.pvt.height_msl_m,
                    velocity_mps: -sample.pvt.velocity_down_mps,
                    height_std_m: (sample.pvt.vertical_accuracy_mm as f32 / 1_000.0)
                        .max(GNSS_MIN_STD),
                    velocity_std_mps: sample.pvt.speed_accuracy_mps.max(GNSS_MIN_STD),
                },
                // SEF-light's convention: 3 is a usable 3D fix; it ignores tiers below.
                fix_tier: match sample.pvt.fix_type {
                    _ if !sample.pvt.fix_ok => 0,
                    ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning => 3,
                    ublox::GpsFix::Fix2D => 2,
                    _ => 0,
                },
                pdop_centi: sample.pvt.pdop_centi,
            })
        });
        if let Some(updates) = self.estimator.update_gnss(epoch.ts.as_micros(), samples)? {
            for (ready, update) in self.msl_ready.iter_mut().zip(updates) {
                *ready |= update.height.accepted;
            }
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

        let selected = imu.index();
        let state = self.estimator.selected_state();
        let uncertainty = self.estimator.selected_uncertainty();
        signals::STATE_ESTIMATE_WATCH.sender().send(StateEstimate {
            msl_ready: self.msl_ready[selected],
            height_msl_m: state.height_m,
            velocity_mps: state.velocity_mps,
            height_std_m: libm::sqrtf(uncertainty.height_variance_m2),
            velocity_std_mps: libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
            orientation_body_to_ned_wxyz: self.estimator.orientation_body_to_ned_wxyz(imu),
            selected_imu: ImuId::ALL[selected],
            redundancy_ready: self.estimator.redundancy_ready(),
        });

        let scores = self.estimator.consistency_scores();
        for (id, imu) in ImuId::ALL.into_iter().zip(asteria_sef_light::ImuId::ALL) {
            let state = self.estimator.state(imu);
            let uncertainty = self.estimator.uncertainty(imu);
            signals::submit_state(SefLogSample {
                ts,
                imu: id,
                selected: imu.index() == selected,
                msl_ready: self.msl_ready[imu.index()],
                redundancy_ready: self.estimator.redundancy_ready(),
                height_msl_m: state.height_m,
                velocity_mps: state.velocity_mps,
                barometer_bias_m: state.barometer_bias_m,
                height_std_m: libm::sqrtf(uncertainty.height_variance_m2),
                velocity_std_mps: libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
                barometer_bias_std_m: uncertainty.barometer_bias_variance_m2.map(libm::sqrtf),
                consistency_score: scores[imu.index()],
                orientation_body_to_ned_wxyz: self.estimator.orientation_body_to_ned_wxyz(imu),
            });
        }
        self.last_output = Some(now);
    }
}
