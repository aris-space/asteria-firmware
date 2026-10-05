//! SEF-light vertical estimation from the calibrated sensor streams.
//!
//! Samples are held for [`HOLDBACK`] and handed to the estimator in timestamp
//! order. SEF-light accepts late samples, but each one replays its history, and
//! IMU samples arrive in FIFO batches that are already up to ~15 ms old.
//! Each output tick publishes the selected chain to
//! [`signals::STATE_ESTIMATE_WATCH`] and logs every chain to SD.

use asteria_sef_light::{EstimatorError, PressureMeasurement};
use defmt::{Debug2Format, warn};
use embassy_futures::select::{Either, Either4, select, select4};
use embassy_time::{Duration, Instant, Timer};
use heapless::Vec;

use crate::calibration;
use crate::sef::{
    Estimator, GnssVerticalInput, barometric_pressure_altitude_m, gnss_measurement,
    imu_measurement, new_estimator,
};
use crate::sensors::{GNSS_COUNT, IMU_COUNT, ImuId};
use crate::signals;
use crate::tasks::readout::imu::GYRO_RANGE_DPS;
use crate::types::{
    BaroSample, GnssSample, ImuSample, MagSample, SdLogRecord, SefLogSample, StateEstimate,
};

const HOLDBACK: Duration = Duration::from_millis(35);
// Two IMUs at 833 Hz fill about 60 slots during the holdback.
const PENDING_CAPACITY: usize = 128;
const OUTPUT_PERIOD: Duration = Duration::from_millis(50);
const IMU_FRESH: Duration = Duration::from_millis(100);
const WARNING_PERIOD: Duration = Duration::from_secs(1);
// Both stationary barometers varied by about 0.3 m in the bench run; this
// larger uncertainty allows for pressure changes and correlated samples.
const BARO_HEIGHT_STD_M: f32 = 1.5;

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
    // Samples waiting out the holdback, in arrival order.
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
            Either::Second(Either4::First(sample)) => Event::Imu(sample),
            Either::Second(Either4::Second(sample)) => Event::Mag(sample),
            Either::Second(Either4::Third(sample)) => Event::Gnss(sample),
            Either::Second(Either4::Fourth(sample)) => Event::Baro(sample),
        };
        if pending.push(event).is_err() {
            warn!("SEF: input buffer full, dropped a sample");
        }
    }
}

#[derive(Clone, Copy)]
enum Event {
    Imu(ImuSample),
    Baro(BaroSample),
    Mag(MagSample),
    Gnss(GnssSample),
}

impl Event {
    fn ts(self) -> Instant {
        match self {
            Self::Imu(sample) => sample.ts,
            Self::Baro(sample) => sample.ts,
            Self::Mag(sample) => sample.ts,
            Self::Gnss(sample) => sample.ts,
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
    estimator: Estimator,
    gnss_epoch: Option<GnssEpoch>,
    /// Per chain: a GNSS height update was accepted, so its height is MSL.
    msl_ready: [bool; IMU_COUNT],
    last_output: Option<Instant>,
    last_warning: Option<Instant>,
}

impl Processor {
    fn new() -> Result<Self, EstimatorError> {
        Ok(Self {
            estimator: new_estimator(GYRO_RANGE_DPS)?,
            gnss_epoch: None,
            msl_ready: [false; IMU_COUNT],
            last_output: None,
            last_warning: None,
        })
    }

    fn handle(&mut self, event: Event) {
        let result = match event {
            Event::Imu(sample) => self.update_imu(sample),
            Event::Baro(sample) => self.update_baro(sample),
            Event::Mag(sample) => self.update_mag(sample),
            Event::Gnss(sample) => self.update_gnss(sample),
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

    fn update_baro(&mut self, sample: BaroSample) -> Result<(), EstimatorError> {
        let height_m = barometric_pressure_altitude_m(sample.pressure_mbar)
            .ok_or(EstimatorError::OutOfRangeInput)?;
        let barometer = asteria_sef_light::BarometerId::from_index(sample.src.index())
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

    fn update_mag(&mut self, sample: MagSample) -> Result<(), EstimatorError> {
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
        self.estimator.update_magnetometer(
            sef_imu(sample.src.index()),
            sample.ts.as_micros(),
            field,
        )
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
            sample.map(|sample| {
                gnss_measurement(GnssVerticalInput {
                    height_msl_m: sample.pvt.height_msl_m,
                    velocity_down_mps: sample.pvt.velocity_down_mps,
                    vertical_accuracy_mm: sample.pvt.vertical_accuracy_mm,
                    speed_accuracy_mps: sample.pvt.speed_accuracy_mps,
                    fix_tier: fix_tier(sample.pvt.fix_type),
                    pdop_centi: sample.pvt.pdop_centi,
                })
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
            ts,
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
        for id in ImuId::ALL {
            let index = id.index();
            let imu = sef_imu(index);
            let state = self.estimator.state(imu);
            let uncertainty = self.estimator.uncertainty(imu);
            signals::submit_sd_log(SdLogRecord::State(SefLogSample {
                ts,
                imu: id,
                selected: index == selected,
                msl_ready: self.msl_ready[index],
                redundancy_ready: self.estimator.redundancy_ready(),
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

/// SEF-light's fix tier convention: 3 is a usable 3D fix.
fn fix_tier(fix_type: ublox::GpsFix) -> u8 {
    match fix_type {
        ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning => 3,
        ublox::GpsFix::Fix2D => 2,
        _ => 0,
    }
}
