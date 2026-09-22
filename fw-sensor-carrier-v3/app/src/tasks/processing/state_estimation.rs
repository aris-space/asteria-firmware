//! SEF-light vertical estimation from the calibrated sensor streams.

use core::future::pending;

use asteria_sef_light::{
    DualVerticalEstimator, EstimatorError, GnssSample as FilterGnssSample, GnssSelectorConfig,
    ImuAttitudeConfig, ImuMeasurement, PressureMeasurement, SelectorConfig,
    VerticalEstimatorSelectorConfig, VerticalFilterConfig, VerticalGnssMeasurement,
};
use defmt::{Debug2Format, warn};
use embassy_futures::select::{Either, Either6, select, select6};
use embassy_time::{Duration, Instant, Timer};

use crate::sensors::{BARO_BUS_1, BARO_BUS_2, GNSS_0, GNSS_1, IMU_0, IMU_1};
use crate::signals;
use crate::tasks::readout::imu::GYRO_RANGE_DPS;
use crate::types::{BaroSample, GnssSample, ImuSample, VerticalEstimate};

// Two 833 Hz IMUs produce about 667 events in 400 ms. The remaining capacity
// covers barometers, GNSS, and interrupt scheduling jitter.
const HISTORY_CAPACITY: usize = 768;
const MAX_AIDING_DELAY_US: u64 = 400_000;
const GNSS_PAIR_WAIT: Duration = Duration::from_millis(150);
const OUTPUT_PERIOD: Duration = Duration::from_millis(50);
const IMU_FRESH: Duration = Duration::from_millis(100);
const BARO_HEIGHT_STD_M: f32 = 3.0;

type Estimator = DualVerticalEstimator<HISTORY_CAPACITY>;

enum Event {
    Imu(ImuSample),
    Barometer(BaroSample),
    Gnss(GnssSample),
    GnssTimeout,
}

struct PendingGnss {
    sample: GnssSample,
    deadline: Instant,
}

struct Processor {
    estimator: Estimator,
    baro_reference: [Option<(f32, f32)>; 2],
    launch_height_msl_m: Option<f32>,
    pending_gnss: Option<PendingGnss>,
    last_output: Option<Instant>,
    last_warning: Option<Instant>,
}

impl Processor {
    fn new() -> Result<Self, EstimatorError> {
        let filter = VerticalFilterConfig::new(0.5, 5.0, [0.02, 0.02], 10.0, 3.0, [5.0, 5.0], 5.0)?;
        let attitude = ImuAttitudeConfig::new(2.0, GYRO_RANGE_DPS, 10.0, 300)?;
        let selection = SelectorConfig::new(2.0, 250_000).ok_or(EstimatorError::OutOfRangeInput)?;
        let selector = VerticalEstimatorSelectorConfig::new(0.95, 25.0, 10.0, 100_000, selection)?;
        let gnss =
            GnssSelectorConfig::new(3, 4.0, 500_000).ok_or(EstimatorError::OutOfRangeInput)?;
        Ok(Self {
            estimator: Estimator::new(filter, [attitude; 2], selector, gnss, MAX_AIDING_DELAY_US)?,
            baro_reference: [None; 2],
            launch_height_msl_m: None,
            pending_gnss: None,
            last_output: None,
            last_warning: None,
        })
    }

    fn handle(&mut self, event: Event) {
        let result = match event {
            Event::Imu(sample) => self.update_imu(sample),
            Event::Barometer(sample) => self.update_barometer(sample),
            Event::Gnss(sample) => self.queue_gnss(sample),
            Event::GnssTimeout => self.flush_gnss(),
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
    }

    fn update_imu(&mut self, sample: ImuSample) -> Result<(), EstimatorError> {
        let imu = asteria_sef_light::ImuId::from_index(sample.src.index())
            .expect("firmware IMU ID must map to SEF-light");
        const G: f32 = asteria_sef_light::STANDARD_GRAVITY_MPS2;
        const DPS_TO_RAD: f32 = core::f32::consts::PI / 180.0;
        let measurement = ImuMeasurement {
            acceleration_body_mps2: [sample.accel.x * G, sample.accel.y * G, sample.accel.z * G],
            angular_rate_body_rad_s: [
                sample.gyro.x * DPS_TO_RAD,
                sample.gyro.y * DPS_TO_RAD,
                sample.gyro.z * DPS_TO_RAD,
            ],
        };
        self.estimator
            .update_imu(imu, sample.ts.as_micros(), measurement)?;
        Ok(())
    }

    fn update_barometer(&mut self, sample: BaroSample) -> Result<(), EstimatorError> {
        let index = sample.src.index();
        let pressure = sample.pressure_mbar;
        let temperature_k = sample.temperature_c + 273.15;
        if !pressure.is_finite()
            || pressure <= 0.0
            || !temperature_k.is_finite()
            || temperature_k <= 0.0
        {
            return Err(EstimatorError::OutOfRangeInput);
        }
        let (reference_pressure, reference_temperature_k) =
            *self.baro_reference[index].get_or_insert((pressure, temperature_k));
        // Hypsometric conversion relative to each sensor's launch pressure.
        let height_m = 29.271 * reference_temperature_k * libm::logf(reference_pressure / pressure);
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

    fn queue_gnss(&mut self, sample: GnssSample) -> Result<(), EstimatorError> {
        if sample.pvt.height_msl.is_finite()
            && self.launch_height_msl_m.is_none()
            && matches!(
                sample.pvt.fix_type,
                ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning
            )
        {
            self.launch_height_msl_m = Some(sample.pvt.height_msl);
        }
        if let Some(pending) = self.pending_gnss.take() {
            if sample.src == pending.sample.src
                && sample.pvt.epoch_ms == pending.sample.pvt.epoch_ms
            {
                self.pending_gnss = Some(pending);
                return Ok(());
            }
            let span = if sample.ts >= pending.sample.ts {
                sample.ts - pending.sample.ts
            } else {
                pending.sample.ts - sample.ts
            };
            if sample.src != pending.sample.src
                && sample.pvt.epoch_ms == pending.sample.pvt.epoch_ms
                && span <= GNSS_PAIR_WAIT
            {
                let mut pair = [None, None];
                pair[pending.sample.src.index()] = self.gnss_measurement(pending.sample);
                pair[sample.src.index()] = self.gnss_measurement(sample);
                let fusion_time_us = sample.ts.as_micros().min(pending.sample.ts.as_micros());
                self.estimator.update_gnss(fusion_time_us, pair)?;
                return Ok(());
            }
            let result = self.fuse_single(pending.sample);
            self.pending_gnss = Some(PendingGnss {
                sample,
                deadline: Instant::now() + GNSS_PAIR_WAIT,
            });
            return result;
        }
        self.pending_gnss = Some(PendingGnss {
            sample,
            deadline: Instant::now() + GNSS_PAIR_WAIT,
        });
        Ok(())
    }

    fn flush_gnss(&mut self) -> Result<(), EstimatorError> {
        if let Some(pending) = self.pending_gnss.take() {
            self.fuse_single(pending.sample)?;
        }
        Ok(())
    }

    fn fuse_single(&mut self, sample: GnssSample) -> Result<(), EstimatorError> {
        let mut pair = [None, None];
        pair[sample.src.index()] = self.gnss_measurement(sample);
        self.estimator.update_gnss(sample.ts.as_micros(), pair)?;
        Ok(())
    }

    fn gnss_measurement(
        &self,
        sample: GnssSample,
    ) -> Option<FilterGnssSample<VerticalGnssMeasurement>> {
        let origin = self.launch_height_msl_m?;
        let fix_tier = match sample.pvt.fix_type {
            ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning => 3,
            _ => 0,
        };
        let height_std_m = (sample.pvt.vert_accuracy as f32 / 1000.0).max(0.5);
        let velocity_std_mps = if sample.pvt.speed_accuracy_mps > 0.0 {
            sample.pvt.speed_accuracy_mps.max(0.1)
        } else {
            1.0
        };
        Some(FilterGnssSample {
            measurement: VerticalGnssMeasurement {
                height_m: sample.pvt.height_msl - origin,
                velocity_mps: -sample.pvt.vel_down,
                height_std_m,
                velocity_std_mps,
            },
            fix_tier,
            pdop_centi: sample.pvt.pdop,
        })
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
        let state = self.estimator.selected_state();
        let uncertainty = self.estimator.selected_uncertainty();
        let selected_imu = if imu.index() == 0 { IMU_0 } else { IMU_1 };
        signals::VERTICAL_ESTIMATE_WATCH
            .sender()
            .send(VerticalEstimate {
                ts,
                height_m: state.height_m,
                velocity_mps: state.velocity_mps,
                height_std_m: libm::sqrtf(uncertainty.height_variance_m2),
                velocity_std_mps: libm::sqrtf(uncertainty.velocity_variance_m2_per_s2),
                selected_imu,
                redundancy_ready: self.estimator.redundancy_ready(),
            });
        self.last_output = Some(now);
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

    loop {
        if processor
            .pending_gnss
            .as_ref()
            .is_some_and(|pending| Instant::now() >= pending.deadline)
        {
            processor.handle(Event::GnssTimeout);
            continue;
        }
        let deadline = processor
            .pending_gnss
            .as_ref()
            .map(|pending| pending.deadline);
        let samples = select6(
            baro0.next_message_pure(),
            baro1.next_message_pure(),
            gnss0.next_message_pure(),
            gnss1.next_message_pure(),
            imu0.next_message_pure(),
            imu1.next_message_pure(),
        );
        let timeout = async {
            if let Some(deadline) = deadline {
                Timer::at(deadline).await;
            } else {
                pending::<()>().await;
            }
        };
        let event = match select(samples, timeout).await {
            Either::First(Either6::First(sample) | Either6::Second(sample)) => {
                Event::Barometer(sample)
            }
            Either::First(Either6::Third(sample) | Either6::Fourth(sample)) => Event::Gnss(sample),
            Either::First(Either6::Fifth(sample) | Either6::Sixth(sample)) => Event::Imu(sample),
            Either::Second(()) => Event::GnssTimeout,
        };
        processor.handle(event);
    }
}
