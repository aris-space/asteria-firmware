//! SEF-light vertical estimation from the calibrated sensor streams.

use core::future::pending;

use asteria_sef_light::{
    EstimatorError, GnssSample as FilterGnssSample, PressureMeasurement, VerticalGnssMeasurement,
};
use defmt::{Debug2Format, info, warn};
use embassy_futures::select::{Either, Either6, select, select6};
use embassy_time::{Duration, Instant, Timer};
use fw_sensor_carrier_v3::sef::{
    BarometerReference, Estimator, GnssEpoch, GnssVerticalInput, gnss_measurement, imu_measurement,
    new_estimator, pairable_epoch,
};

use crate::sensors::{BARO_BUS_1, BARO_BUS_2, GNSS_0, GNSS_1, IMU_0, IMU_1};
use crate::signals;
use crate::tasks::readout::imu::GYRO_RANGE_DPS;
use crate::types::{BaroSample, GnssSample, ImuSample, VerticalEstimate};

const GNSS_PAIR_WAIT: Duration = Duration::from_millis(150);
const OUTPUT_PERIOD: Duration = Duration::from_millis(50);
const IMU_FRESH: Duration = Duration::from_millis(100);
const BARO_HEIGHT_STD_M: f32 = 3.0;

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
    baro_reference: [Option<BarometerReference>; 2],
    gnss_reference_msl_m: Option<f32>,
    pending_gnss: Option<PendingGnss>,
    last_output: Option<Instant>,
    last_status_log: Option<Instant>,
    last_warning: Option<Instant>,
    published_since_status: u32,
}

impl Processor {
    fn new() -> Result<Self, EstimatorError> {
        Ok(Self {
            estimator: new_estimator(GYRO_RANGE_DPS)?,
            baro_reference: [None; 2],
            gnss_reference_msl_m: None,
            pending_gnss: None,
            last_output: None,
            last_status_log: None,
            last_warning: None,
            published_since_status: 0,
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
        let measurement = imu_measurement(
            [sample.accel.x, sample.accel.y, sample.accel.z],
            [sample.gyro.x, sample.gyro.y, sample.gyro.z],
        );
        self.estimator
            .update_imu(imu, sample.ts.as_micros(), measurement)?;
        Ok(())
    }

    fn update_barometer(&mut self, sample: BaroSample) -> Result<(), EstimatorError> {
        let index = sample.src.index();
        let reference = match self.baro_reference[index] {
            Some(reference) => reference,
            None => {
                let reference = BarometerReference::new(sample.pressure_mbar, sample.temperature_c)
                    .ok_or(EstimatorError::OutOfRangeInput)?;
                self.baro_reference[index] = Some(reference);
                reference
            }
        };
        let height_m = reference
            .height_m(sample.pressure_mbar)
            .ok_or(EstimatorError::OutOfRangeInput)?;
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
            && self.gnss_reference_msl_m.is_none()
            && matches!(
                sample.pvt.fix_type,
                ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning
            )
        {
            // Match the GNSS MSL frame to the barometer/IMU frame even if the
            // first GNSS fix arrives after the board has moved.
            self.gnss_reference_msl_m =
                Some(sample.pvt.height_msl - self.estimator.selected_state().height_m);
        }
        if let Some(pending) = self.pending_gnss.take() {
            if sample.src == pending.sample.src
                && sample.pvt.epoch_ms == pending.sample.pvt.epoch_ms
            {
                self.pending_gnss = Some(pending);
                return Ok(());
            }
            if pairable_epoch(
                GnssEpoch {
                    receiver: pending.sample.src.index(),
                    epoch_ms: pending.sample.pvt.epoch_ms,
                    time_us: pending.sample.ts.as_micros(),
                },
                GnssEpoch {
                    receiver: sample.src.index(),
                    epoch_ms: sample.pvt.epoch_ms,
                    time_us: sample.ts.as_micros(),
                },
                GNSS_PAIR_WAIT.as_micros(),
            ) {
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
        let origin = self.gnss_reference_msl_m?;
        let fix_tier = match sample.pvt.fix_type {
            ublox::GpsFix::Fix3D | ublox::GpsFix::GPSPlusDeadReckoning => 3,
            _ => 0,
        };
        Some(gnss_measurement(GnssVerticalInput {
            launch_height_msl_m: origin,
            height_msl_m: sample.pvt.height_msl,
            velocity_down_mps: sample.pvt.vel_down,
            vertical_accuracy_mm: sample.pvt.vert_accuracy,
            speed_accuracy_mps: sample.pvt.speed_accuracy_mps,
            fix_tier,
            pdop_centi: sample.pvt.pdop,
        }))
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
        self.published_since_status = self.published_since_status.saturating_add(1);
        if self
            .last_status_log
            .is_none_or(|last| now.saturating_duration_since(last) >= Duration::from_secs(1))
        {
            info!(
                "SEF-light: h={}±{} m, v={}±{} m/s, IMU={}, redundancy_ready={}, published={}",
                state.height_m,
                libm::sqrtf(uncertainty.height_variance_m2),
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
