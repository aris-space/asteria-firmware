//! LSM6DSO32 IMU readout.
//!
//! Like every readout in this module, the task follows the shared
//! `Inactive` <-> `Active` shape (see [`super`]): `Inactive` tries to
//! init the sensor and retries with exponential backoff on failure;
//! `Active` runs the FIFO drain loop until too many consecutive errors
//! push it back to `Inactive`. Each transition flips
//! [`crate::sensors::IMU_STATUS`] so we can publish the state to CAN.
//!
//! The sensor batches accel and gyro samples into its hardware FIFO at the
//! configured ODR. Each FIFO entry carries a `TagSensor` byte that says
//! whether it's an accel or gyro reading; because [`ACCEL_ODR`] and [`GYRO_ODR`]
//! are the same, we expect them to be produced in pairs. While it remains undefined
//! in the datasheet, in practice these pairs are interleaved.
//!
//! One of the interrupt lines (INT1) is configured to fire when the FIFO
//! crosses [`FIFO_WATERMARK`]. We then drain whatever is queued into a local
//! scratch buffer ([`FIFO_BUFFER_SIZE`] entries) with a single SPI transaction, iterate
//! it as accel+gyro pairs, apply the sensor-to-board axis flip, and publish one [`ImuSample`]
//! per pair. Sample timestamps are interpolated across the batch using the wall-clock
//! interval between consecutive interrupts.
//!
//! [`LOOP_TIMEOUT`] bounds the wait on INT1 so a missed interrupt is
//! recovered after roughly two expected periods.

use core::sync::atomic::Ordering;

use defmt::{Debug2Format, debug, error, info, trace, warn};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use lsm6dso32::spi::Lsm6Dso32SpiInterface;
use lsm6dso32::{
    AccelBatchDataRate, Acceleration, AccelerationRaw, AccelerometerFullScale, AccelerometerOdr,
    AngularRate, AngularRateRaw, FifoDataOut, FifoMode, GyroBatchDataRate, GyroscopeFullScale,
    GyroscopeOdr, Initialised, Int1Config, Lsm6dso32, TagSensor, Uninitialised,
};

use super::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::resources::sensors::SpiDevice;
use crate::sensors::{IMU_STATUS, ImuId, SensorStatus};
use crate::signals;
use crate::types::ImuSample;

/// Accelerometer output data rate. Should match [`IMU_ODR_HZ`]
const ACCEL_ODR: AccelerometerOdr = AccelerometerOdr::Hz833;
/// Gyroscope output data rate. Should match [`IMU_ODR_HZ`]
const GYRO_ODR: GyroscopeOdr = GyroscopeOdr::Hz833;
/// Accelerometer batch data rate. Should match [`ACCEL_ODR`]
const ACCEL_BDR: AccelBatchDataRate = AccelBatchDataRate::Hz833;
/// Gyroscope batch data rate. Should match [`GYRO_ODR`]
const GYRO_BDR: GyroBatchDataRate = GyroBatchDataRate::Hz833;
/// IMU output data rate in Hz. MUST match the above ODR and BDR settings.
/// Do not change without also updating the above settings!
pub const IMU_ODR_HZ: u32 = 833;
/// Target IMU loop data rate
pub const IMU_TARGET_DT: f32 = 1.0 / IMU_ODR_HZ as f32;
/// Accelerometer full-scale range
const ACCEL_FULL_SCALE: AccelerometerFullScale = AccelerometerFullScale::G8;
/// Gyroscope full-scale range
const GYRO_FULL_SCALE: GyroscopeFullScale = GyroscopeFullScale::Dps2000;
pub const GYRO_RANGE_DPS: f32 = match GYRO_FULL_SCALE {
    GyroscopeFullScale::Dps250 => 250.0,
    GyroscopeFullScale::Dps500 => 500.0,
    GyroscopeFullScale::Dps1000 => 1000.0,
    GyroscopeFullScale::Dps2000 => 2000.0,
};

/// Local scratch buffer for FIFO drains. Sized larger than the expected
/// per-interrupt batch; the sensor's hardware FIFO is independent.
const FIFO_BUFFER_SIZE: usize = 512;
/// FIFO watermark in entries. With paired accel+gyro at 833 Hz, the
/// watermark interrupt fires every ~15.6 ms (13 pairs).
const FIFO_WATERMARK: u16 = 26;
/// Max wait for the FIFO watermark interrupt before retrying. Roughly
/// 2x the expected period, to catch a missed/stuck interrupt.
const LOOP_TIMEOUT: Duration = Duration::from_millis(30);

async fn configure<SPI: embedded_hal_async::spi::SpiDevice>(
    sensor: &mut Lsm6dso32<Lsm6Dso32SpiInterface<SPI>, Initialised>,
) -> Result<(), ()> {
    sensor
        .set_accelerometer_odr_and_full_scale(Some(ACCEL_ODR), Some(ACCEL_FULL_SCALE))
        .await
        .map_err(|_| ())?;
    sensor
        .set_gyroscope_odr_and_full_scale(Some(GYRO_ODR), Some(GYRO_FULL_SCALE))
        .await
        .map_err(|_| ())?;
    sensor.set_fifo_mode(FifoMode::Fifo).await.map_err(|_| ())?;
    sensor
        .set_fifo_batch_data_rates(
            Some(ACCEL_BDR),
            Some(GYRO_BDR),
            Some(lsm6dso32::TempBatchDataRate::NotBatched),
            Some(lsm6dso32::DecTsBatch::NotBatched),
        )
        .await
        .map_err(|_| ())?;
    sensor
        .configure_fifo(FIFO_WATERMARK, false)
        .await
        .map_err(|_| ())?;
    sensor
        .configure_interrupts(
            Some(Int1Config {
                fifo_th: true,
                ..Default::default()
            }),
            None,
        )
        .await
        .map_err(|_| ())?;
    Ok(())
}

struct Inactive<SPI, INT> {
    iface: Lsm6Dso32SpiInterface<SPI>,
    int1: INT,
    id: ImuId,
    attempt: u8,
}

impl<SPI: embedded_hal_async::spi::SpiDevice, INT: embedded_hal_async::digital::Wait>
    Inactive<SPI, INT>
{
    async fn run(mut self) -> Active<SPI, INT> {
        loop {
            debug!("{} initializing", self.id);
            let uninit = Lsm6dso32::<_, Uninitialised>::new(self.iface);
            match uninit.init(&mut Delay).await {
                Ok(mut sensor) => match configure(&mut sensor).await {
                    Ok(()) => {
                        info!("{} initialized", self.id);
                        return Active {
                            sensor,
                            int1: self.int1,
                            id: self.id,
                        };
                    }
                    Err(()) => {
                        error!("{} configuration failed", self.id);
                        self.iface = sensor.destroy();
                    }
                },
                Err(err) => {
                    error!("{} init failed: {:?}", self.id, Debug2Format(&err.kind));
                    self.iface = err.sensor.destroy();
                }
            }
            self.attempt = self.attempt.saturating_add(1);
            Timer::after(backoff(self.attempt)).await;
        }
    }
}

struct Active<SPI, INT> {
    sensor: Lsm6dso32<Lsm6Dso32SpiInterface<SPI>, Initialised>,
    int1: INT,
    id: ImuId,
}

impl<SPI: embedded_hal_async::spi::SpiDevice, INT: embedded_hal_async::digital::Wait>
    Active<SPI, INT>
{
    async fn run(mut self) -> Inactive<SPI, INT> {
        let mut fifo_buf = [FifoDataOut::new_with_zero(); FIFO_BUFFER_SIZE];
        let mut this_data_end = Instant::now();
        let mut errors: u8 = 0;

        loop {
            let _ = with_timeout(LOOP_TIMEOUT, self.int1.wait_for_rising_edge()).await;

            let fifo_level = match self.sensor.read_fifo_level().await {
                Ok(level) => level,
                Err(e) => {
                    warn!("{} FIFO level read error: {:?}", self.id, Debug2Format(&e));
                    errors = errors.saturating_add(1);
                    if errors >= MAX_CONSECUTIVE_ERRORS {
                        break;
                    }
                    continue;
                }
            };

            let this_data_start = this_data_end;
            this_data_end = Instant::now();

            let fifo_entries = (fifo_level as usize).min(fifo_buf.len()) & !1;
            if fifo_entries == 0 {
                warn!("{} FIFO empty", self.id);
                errors = errors.saturating_add(1);
                if errors >= MAX_CONSECUTIVE_ERRORS {
                    break;
                }
                continue;
            }

            if let Err(e) = self
                .sensor
                .read_multiple_fifo_data(&mut fifo_buf[..fifo_entries])
                .await
            {
                warn!("{} FIFO data read error: {:?}", self.id, Debug2Format(&e));
                errors = errors.saturating_add(1);
                if errors >= MAX_CONSECUTIVE_ERRORS {
                    break;
                }
                continue;
            }

            let num_pairs = fifo_entries / 2;
            let avg_dt_us =
                this_data_end.duration_since(this_data_start).as_micros() / num_pairs as u64;

            // While the sensor's fifo can in theory produce timestamps, in practice these were
            // less accurate than simply using the wall clock time and interpolating over samples.
            let mut samples = heapless::Vec::<ImuSample, 256>::new();
            for (i, chunk) in fifo_buf[..fifo_entries].chunks_exact(2).enumerate() {
                let (acc, gyr) = match (chunk[0].tag_sensor(), chunk[1].tag_sensor()) {
                    (TagSensor::AccelerometerNC, TagSensor::GyroscopeNC) => (chunk[0], chunk[1]),
                    (TagSensor::GyroscopeNC, TagSensor::AccelerometerNC) => (chunk[1], chunk[0]),
                    other => {
                        warn!(
                            "{} unexpected FIFO tag pair: {:?}",
                            self.id,
                            Debug2Format(&other)
                        );
                        continue;
                    }
                };

                // Convert Sensor -> board frame: flip X and Z
                let accel = Acceleration::from_raw(
                    AccelerationRaw {
                        x: acc.x().saturating_neg(),
                        y: acc.y(),
                        z: acc.z().saturating_neg(),
                    },
                    self.sensor.accel_full_scale(),
                );
                let gyro = AngularRate::from_raw(
                    AngularRateRaw {
                        x: gyr.x().saturating_neg(),
                        y: gyr.y(),
                        z: gyr.z().saturating_neg(),
                    },
                    self.sensor.gyro_full_scale(),
                );

                let ts = this_data_start + Duration::from_micros(avg_dt_us * i as u64);
                let _ = samples.push(ImuSample {
                    src: self.id,
                    ts,
                    accel,
                    gyro,
                });
            }

            signals::submit_imu_sample_batch(&samples);
            errors = 0;
            trace!("{} FIFO {} pairs, dt={} us", self.id, num_pairs, avg_dt_us);
        }

        error!("{} offline (too many consecutive errors)", self.id);
        Inactive {
            iface: self.sensor.destroy(),
            int1: self.int1,
            id: self.id,
            attempt: 0,
        }
    }
}

async fn run_inner<SPI, INT>(spi: SPI, int1: INT, id: ImuId) -> !
where
    SPI: embedded_hal_async::spi::SpiDevice,
    INT: embedded_hal_async::digital::Wait,
{
    let mut inactive = Inactive {
        iface: Lsm6Dso32SpiInterface { spi },
        int1,
        id,
        attempt: 0,
    };

    loop {
        let active = inactive.run().await;
        IMU_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
        inactive = active.run().await;
        IMU_STATUS[id.index()].store(SensorStatus::Inactive, Ordering::Relaxed);
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(spi: SpiDevice, int1: ExtiInput<'static, Async>, id: ImuId) -> ! {
    run_inner(spi, int1, id).await
}
