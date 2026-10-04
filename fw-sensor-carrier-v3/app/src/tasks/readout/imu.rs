//! LSM6DSO32 IMU readout.
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
//! it as accel+gyro pairs, calibrate them, and publish one [`ImuSample`]
//! per pair. Sample timestamps are interpolated across the batch using the wall-clock
//! interval between consecutive interrupts.
//!
//! [`LOOP_TIMEOUT`] bounds the wait on INT1 so a missed interrupt is
//! recovered after roughly two expected periods.

use defmt::{Debug2Format, debug, error, info, warn};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use lsm6dso32::spi::Lsm6Dso32SpiInterface;
use lsm6dso32::{
    AccelBatchDataRate, AccelerationRaw, AccelerometerFullScale, AccelerometerOdr, AngularRateRaw,
    FifoDataOut, FifoMode, GyroBatchDataRate, GyroscopeFullScale, GyroscopeOdr, Initialised,
    Int1Config, Lsm6dso32, TagSensor, Uninitialised,
};

use super::{MAX_CONSECUTIVE_ERRORS, State, backoff};
use crate::calibration;
use crate::resources::sensors::SpiDevice;
use crate::sensors::{IMU_STATUS, ImuId};
use crate::signals;
use crate::types::{ImuSample, RawImuSample, SdLogRecord};

/// Accelerometer output data rate. Keep both ODRs and BDRs at 833 Hz.
const ACCEL_ODR: AccelerometerOdr = AccelerometerOdr::Hz833;
/// Gyroscope output data rate. Keep both ODRs and BDRs at 833 Hz.
const GYRO_ODR: GyroscopeOdr = GyroscopeOdr::Hz833;
/// Accelerometer batch data rate. Should match [`ACCEL_ODR`]
const ACCEL_BDR: AccelBatchDataRate = AccelBatchDataRate::Hz833;
/// Gyroscope batch data rate. Should match [`GYRO_ODR`]
const GYRO_BDR: GyroBatchDataRate = GyroBatchDataRate::Hz833;
/// Accelerometer full-scale range
pub const ACCEL_FULL_SCALE: AccelerometerFullScale = AccelerometerFullScale::G8;
/// Gyroscope full-scale range
pub const GYRO_FULL_SCALE: GyroscopeFullScale = GyroscopeFullScale::Dps2000;
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
// ST AN5473 specifies 70 ms gyro turn-on plus three samples at 833 Hz.
const GYRO_SETTLE_TIME: Duration = Duration::from_millis(75);

async fn configure<SPI: embedded_hal_async::spi::SpiDevice>(
    sensor: &mut Lsm6dso32<Lsm6Dso32SpiInterface<SPI>, Initialised>,
) -> Result<(), ()> {
    // Clear data left in the FIFO across an MCU-only reset before enabling ODR.
    sensor
        .set_fifo_mode(FifoMode::Bypass)
        .await
        .map_err(|_| ())?;
    sensor
        .set_accelerometer_odr_and_full_scale(Some(ACCEL_ODR), Some(ACCEL_FULL_SCALE))
        .await
        .map_err(|_| ())?;
    sensor
        .set_gyroscope_odr_and_full_scale(Some(GYRO_ODR), Some(GYRO_FULL_SCALE))
        .await
        .map_err(|_| ())?;
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
    Timer::after(GYRO_SETTLE_TIME).await;
    sensor.set_fifo_mode(FifoMode::Fifo).await.map_err(|_| ())?;
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

impl<SPI: embedded_hal_async::spi::SpiDevice, INT: embedded_hal_async::digital::Wait> State
    for Inactive<SPI, INT>
{
    type Next = Active<SPI, INT>;

    async fn run(mut self) -> Active<SPI, INT> {
        loop {
            debug!("{} initializing", self.id);
            let uninit = Lsm6dso32::<_, Uninitialised>::new(self.iface);
            match uninit.init(&mut Delay).await {
                Ok(mut sensor) => match configure(&mut sensor).await {
                    Ok(()) => {
                        info!("{} active", self.id);
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

impl<SPI: embedded_hal_async::spi::SpiDevice, INT: embedded_hal_async::digital::Wait> State
    for Active<SPI, INT>
{
    type Next = Inactive<SPI, INT>;

    async fn run(mut self) -> Inactive<SPI, INT> {
        let mut fifo_buf = [FifoDataOut::new_with_zero(); FIFO_BUFFER_SIZE];
        let mut last_read = Instant::now();
        let mut errors: u8 = 0;

        while errors < MAX_CONSECUTIVE_ERRORS {
            let _ = with_timeout(LOOP_TIMEOUT, self.int1.wait_for_rising_edge()).await;
            match self.read_batch(&mut fifo_buf, &mut last_read).await {
                Ok(()) => errors = 0,
                Err(()) => errors += 1,
            }
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

impl<SPI: embedded_hal_async::spi::SpiDevice, INT: embedded_hal_async::digital::Wait>
    Active<SPI, INT>
{
    /// Drains the FIFO and publishes one sample per accel+gyro pair. The
    /// pairs are spread evenly between the previous read and this one; the
    /// sensor's own FIFO timestamps were less accurate than this interpolation.
    async fn read_batch(
        &mut self,
        fifo_buf: &mut [FifoDataOut],
        last_read: &mut Instant,
    ) -> Result<(), ()> {
        let fifo_level = self.sensor.read_fifo_level().await.map_err(|e| {
            warn!("{} FIFO level read error: {:?}", self.id, Debug2Format(&e));
        })?;
        let batch_start = *last_read;
        let read_ts = Instant::now();
        *last_read = read_ts;

        let fifo_entries = (fifo_level as usize).min(fifo_buf.len()) & !1;
        if fifo_entries == 0 {
            warn!("{} FIFO empty", self.id);
            return Err(());
        }
        let fifo = &mut fifo_buf[..fifo_entries];
        self.sensor
            .read_multiple_fifo_data(fifo)
            .await
            .map_err(|e| warn!("{} FIFO data read error: {:?}", self.id, Debug2Format(&e)))?;

        let pair_dt_us =
            read_ts.duration_since(batch_start).as_micros() / (fifo_entries / 2) as u64;
        let mut samples = heapless::Vec::<ImuSample, { FIFO_BUFFER_SIZE / 2 }>::new();
        for (i, pair) in fifo.chunks_exact(2).enumerate() {
            let (acc, gyr) = match (pair[0].tag_sensor(), pair[1].tag_sensor()) {
                (TagSensor::AccelerometerNC, TagSensor::GyroscopeNC) => (pair[0], pair[1]),
                (TagSensor::GyroscopeNC, TagSensor::AccelerometerNC) => (pair[1], pair[0]),
                other => {
                    warn!(
                        "{} unexpected FIFO tag pair: {:?}",
                        self.id,
                        Debug2Format(&other)
                    );
                    continue;
                }
            };
            let raw = RawImuSample {
                src: self.id,
                ts: batch_start + Duration::from_micros(pair_dt_us * i as u64),
                read_ts,
                accel: AccelerationRaw {
                    x: acc.x(),
                    y: acc.y(),
                    z: acc.z(),
                },
                gyro: AngularRateRaw {
                    x: gyr.x(),
                    y: gyr.y(),
                    z: gyr.z(),
                },
            };
            let cal = calibration::imu::apply_calibration(raw);
            let _ = samples.push(cal);
            signals::submit_sd_log(SdLogRecord::Imu { raw, cal });
        }
        signals::submit_imu_sample_batch(&samples);
        Ok(())
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(spi: SpiDevice, int1: ExtiInput<'static, Async>, id: ImuId) -> ! {
    let inactive = Inactive {
        iface: Lsm6Dso32SpiInterface { spi },
        int1,
        id,
        attempt: 0,
    };
    super::run(&IMU_STATUS[id.index()], inactive).await
}
