use defmt::{info, trace, warn};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_time::{Delay, Duration, Instant, Timer, with_timeout};
use lsm6dso32::spi::Lsm6Dso32SpiInterface;
use lsm6dso32::{
    Acceleration, AccelerationRaw, AccelerometerFullScale, AngularRate, AngularRateRaw,
    FifoDataOut, FifoMode, GyroscopeFullScale, Initialised, Int1Config, Lsm6dso32, TagSensor,
    Uninitialised,
};

use crate::tasks::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::measurements::{ImuData, ImuSample, Timestamped};
use crate::resources::sensors::SpiDevice;
use crate::sensors::ImuId;
use crate::signals;

const FIFO_BUFFER_SIZE: usize = 512;
const FIFO_WATERMARK: u16 = 26;
const LOOP_TIMEOUT_MS: u64 = 30;

async fn configure<SPI: embedded_hal_async::spi::SpiDevice>(
    sensor: &mut Lsm6dso32<Lsm6Dso32SpiInterface<SPI>, Initialised>,
) -> Result<(), ()> {
    sensor
        .set_accelerometer_odr_and_full_scale(
            Some(lsm6dso32::AccelerometerOdr::Hz833),
            Some(AccelerometerFullScale::G8),
        )
        .await
        .map_err(|_| ())?;
    sensor
        .set_gyroscope_odr_and_full_scale(
            Some(lsm6dso32::GyroscopeOdr::Hz833),
            Some(GyroscopeFullScale::Dps2000),
        )
        .await
        .map_err(|_| ())?;
    sensor.set_fifo_mode(FifoMode::Fifo).await.map_err(|_| ())?;
    sensor
        .set_fifo_batch_data_rates(
            Some(lsm6dso32::AccelBatchDataRate::Hz833),
            Some(lsm6dso32::GyroBatchDataRate::Hz833),
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
            let uninit = Lsm6dso32::<_, Uninitialised>::new(self.iface);
            match uninit.init(&mut Delay).await {
                Ok(mut sensor) => {
                    if configure(&mut sensor).await.is_err() {
                        warn!("imu: configuration failed, retrying...");
                        self.iface = sensor.destroy();
                        self.attempt = self.attempt.saturating_add(1);
                        Timer::after(backoff(self.attempt)).await;
                        continue;
                    }
                    info!("imu: active");
                    return Active {
                        sensor,
                        int1: self.int1,
                        id: self.id,
                    };
                }
                Err(err) => {
                    self.attempt = self.attempt.saturating_add(1);
                    warn!("imu: init failed (attempt {})", self.attempt);
                    self.iface = err.sensor.destroy();
                    Timer::after(backoff(self.attempt)).await;
                }
            }
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
            let _ = with_timeout(
                Duration::from_millis(LOOP_TIMEOUT_MS),
                self.int1.wait_for_rising_edge(),
            )
            .await;

            let fifo_level = match self.sensor.read_fifo_level().await {
                Ok(level) => level,
                Err(_) => {
                    errors = errors.saturating_add(1);
                    warn!("imu: fifo level read error ({}/{})", errors, MAX_CONSECUTIVE_ERRORS);
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
                errors = errors.saturating_add(1);
                if errors >= MAX_CONSECUTIVE_ERRORS {
                    break;
                }
                continue;
            }

            if self
                .sensor
                .read_multiple_fifo_data(&mut fifo_buf[..fifo_entries])
                .await
                .is_err()
            {
                errors = errors.saturating_add(1);
                warn!("imu: fifo read error ({}/{})", errors, MAX_CONSECUTIVE_ERRORS);
                if errors >= MAX_CONSECUTIVE_ERRORS {
                    break;
                }
                continue;
            }

            let num_pairs = fifo_entries / 2;
            let avg_dt_us =
                this_data_end.duration_since(this_data_start).as_micros() / num_pairs as u64;

            let mut samples = heapless::Vec::<ImuSample, 256>::new();
            for (i, chunk) in fifo_buf[..fifo_entries].chunks_exact(2).enumerate() {
                let (acc, gyr) = match (chunk[0].tag_sensor(), chunk[1].tag_sensor()) {
                    (TagSensor::AccelerometerNC, TagSensor::GyroscopeNC) => (chunk[0], chunk[1]),
                    (TagSensor::GyroscopeNC, TagSensor::AccelerometerNC) => (chunk[1], chunk[0]),
                    _ => continue,
                };

                let accel = Acceleration::from_raw(
                    AccelerationRaw { x: acc.x(), y: acc.y(), z: acc.z() },
                    self.sensor.accel_full_scale(),
                );
                let gyro = AngularRate::from_raw(
                    AngularRateRaw { x: gyr.x(), y: gyr.y(), z: gyr.z() },
                    self.sensor.gyro_full_scale(),
                );

                let ts = this_data_start + Duration::from_micros(avg_dt_us * i as u64);
                let _ = samples.push(ImuSample {
                    sensor_id: self.id,
                    data: Timestamped { ts, value: ImuData { accel, gyro } },
                });
            }

            signals::submit_imu_samples(&samples);
            errors = 0;
            trace!("imu: read {} pairs, dt={} us", num_pairs, avg_dt_us);
        }

        warn!("imu: inactive (too many errors)");
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
        inactive = active.run().await;
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(spi: SpiDevice, int1: ExtiInput<'static, Async>, id: ImuId) -> ! {
    run_inner(spi, int1, id).await
}
