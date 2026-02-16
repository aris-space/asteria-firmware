#![allow(unused)]

use crate::drivers::inertial;
use crate::drivers::inertial::{ImuMeasurement, InertialDriver, INERTIAL_DRIVER};
use crate::sensors::{update_status, CommonSensorConfig, SensorId, SensorStatus};
use crate::util::ExponentialBackoff;
use crate::Debug2Format;
use core::convert::Infallible;
use core::future::pending;
use embassy_executor::task;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::spi::Error;
use embassy_stm32::time::mhz;
use embassy_time::{with_timeout, Delay, Duration, Instant, TimeoutError};
use embedded_utils::fmt::*;
use embedded_utils::ExtendTime;
use heapless::Deque;
use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};
use lsm6dso32::{
    Acceleration, AccelerationRaw, AccelerometerFullScale, AngularRate, AngularRateRaw,
    FifoDataOut, FifoMode, GyroscopeFullScale, Initialised, Int1Config, Lsm6dso32, TagSensor,
    Uninitialised,
};

type Interface = lsm6dso32::spi::Lsm6Dso32SpiInterface<
    embedded_hal_bus::spi::ExclusiveDevice<
        embassy_stm32::spi::Spi<'static, embassy_stm32::mode::Async>,
        embassy_stm32::gpio::Output<'static>,
        Delay,
    >,
>;

type DeviceError = embedded_hal_bus::spi::DeviceError<Error, Infallible>;

// Constants
pub const IMU_ODR_HZ: u32 = 833;
pub const IMU_TARGET_DT: f32 = 1.0 / IMU_ODR_HZ as f32;
const FIFO_BUFFER_SIZE: usize = 512;
const FIFO_WATERMARK: u16 = 26;

// ---------------------------------------------------------
// InactiveImuSensor and ActiveImuSensor states
// ---------------------------------------------------------
struct InactiveImuSensor<'a> {
    driver: &'a InertialDriver<'a>,
    iface: Interface,
    int: ExtiInput<'static>,
    config: CommonSensorConfig,
    sensor_id: SensorId,
    attempt_count: u8,
}

impl<'a> InactiveImuSensor<'a> {
    pub fn new(
        driver: &'a InertialDriver<'a>,
        iface: Interface,
        int: ExtiInput<'static>,
        config: CommonSensorConfig,
        sensor_id: SensorId,
    ) -> Self {
        Self {
            driver,
            iface,
            int,
            config,
            sensor_id,
            attempt_count: 0,
        }
    }

    pub async fn run(mut self) -> ActiveImuSensor<'a> {
        loop {
            debug!("{:?} IMU initializing", self.sensor_id);
            let sensor = Lsm6dso32::<Interface, Uninitialised>::new(self.iface);

            match sensor.init(&mut Delay).await {
                Ok(mut sensor_init) => {
                    let res = configure_imu(&mut sensor_init).await;
                    info!("{:?} IMU initialized", self.sensor_id);
                    return ActiveImuSensor::new(
                        self.driver,
                        sensor_init,
                        self.int,
                        self.config,
                        self.sensor_id,
                    );
                }
                Err(err) => {
                    // Reclaim the interface from the failed sensor
                    self.iface = err.sensor.destroy();
                }
            }

            self.attempt_count += 1;
            let backoff =
                ExponentialBackoff::new(self.config.base_backoff_ms, self.config.max_backoff_ms);
            backoff.wait(self.attempt_count).await;
        }
    }
}

async fn configure_imu(sensor: &mut Lsm6dso32<Interface, Initialised>) -> Result<(), DeviceError> {
    sensor
        .set_accelerometer_odr_and_full_scale(
            Some(lsm6dso32::AccelerometerOdr::Hz833),
            Some(AccelerometerFullScale::G8),
        )
        .await?;
    sensor
        .set_gyroscope_odr_and_full_scale(
            Some(lsm6dso32::GyroscopeOdr::Hz833),
            Some(GyroscopeFullScale::Dps2000),
        )
        .await?;

    // FIFO in FIFO mode
    sensor.set_fifo_mode(FifoMode::Fifo).await?;
    sensor
        .set_fifo_batch_data_rates(
            Some(lsm6dso32::AccelBatchDataRate::Hz833),
            Some(lsm6dso32::GyroBatchDataRate::Hz833),
            Some(lsm6dso32::TempBatchDataRate::NotBatched),
            Some(lsm6dso32::DecTsBatch::NotBatched),
        )
        .await?;

    sensor.configure_fifo(FIFO_WATERMARK, false).await?;
    sensor
        .configure_interrupts(
            Some(Int1Config {
                fifo_th: true,
                ..Default::default()
            }),
            None,
        )
        .await?;

    Ok(())
}

const LOOP_WAIT_MS: u64 = 30;

pub struct ActiveImuSensor<'a> {
    driver: &'a InertialDriver<'a>,
    sensor: Lsm6dso32<Interface, Initialised>,
    int: ExtiInput<'static>,
    config: CommonSensorConfig,
    sensor_id: SensorId,
    error_count: u8,
}

impl<'a> ActiveImuSensor<'a> {
    pub fn new(
        orientation_driver: &'a InertialDriver<'a>,
        sensor: Lsm6dso32<Interface, Initialised>,
        int: ExtiInput<'static>,
        config: CommonSensorConfig,
        sensor_id: SensorId,
    ) -> Self {
        Self {
            driver: orientation_driver,
            sensor,
            int,
            config,
            sensor_id,
            error_count: 0,
        }
    }

    async fn run(mut self) -> InactiveImuSensor<'a> {
        let mut fifo_data_out = [FifoDataOut::new_with_zero(); FIFO_BUFFER_SIZE];

        let mut last_loop_time = Instant::now();
        let mut this_data_end = Instant::now();
        let mut this_data_start;
        let mut last_system_timestamp: Option<Instant> = None;
        let mut error_count: u8 = 0;

        loop {
            let start_wait = Instant::now();
            if with_timeout(
                Duration::from_millis(LOOP_WAIT_MS),
                self.int.wait_for_rising_edge(),
            )
            .await
            .is_err()
            {
                let wait_duration = Instant::now().duration_since(start_wait);
                warn!(
                    "{:?} IMU task was woken with timeout. Waited {}ms, but timeout set to {}ms",
                    self.sensor_id,
                    wait_duration.as_secs_f32() * 1000.0,
                    LOOP_WAIT_MS
                );
            }

            let now = Instant::now();
            let loop_interval = now.duration_since(last_loop_time);
            last_loop_time = now;
            trace!(
                "{:?} Loop interval: {} ms",
                self.sensor_id,
                loop_interval.as_secs_f32() * 1000.0
            );

            let read_start = Instant::now();
            let fifo_level = match self.sensor.read_fifo_level().await {
                Ok(level) => level,
                Err(e) => {
                    error!(
                        "{:?} FIFO level read error: {:?}",
                        self.sensor_id,
                        Debug2Format(&e)
                    );
                    error_count += 1;
                    if error_count >= self.config.max_consecutive_errors {
                        error!("{:?} IMU sensor offline (too many errors)", self.sensor_id);
                        break;
                    }
                    continue;
                }
            };

            // first iteration usually has unstable timestamps
            this_data_start = this_data_end;
            this_data_end = Instant::now();

            #[inline]
            fn floor_even(x: usize) -> usize {
                x & !1
            }

            let fifo_entries = floor_even((fifo_level as usize).min(fifo_data_out.len()));
            if fifo_entries == 0 {
                // This usually only happens if we read too fast or too slow.
                warn!("{:?} FIFO empty or not enough data", self.sensor_id);
                error_count += 1;
                if error_count >= self.config.max_consecutive_errors {
                    error!("{:?} IMU sensor offline (too many errors)", self.sensor_id);
                    break;
                }
                continue;
            }

            if let Err(e) = self
                .sensor
                .read_multiple_fifo_data(&mut fifo_data_out[..fifo_entries])
                .await
            {
                error!(
                    "{:?} FIFO data read error: {:?}",
                    self.sensor_id,
                    Debug2Format(&e)
                );
                error_count += 1;
                if error_count >= self.config.max_consecutive_errors {
                    error!("{:?} IMU sensor offline (too many errors)", self.sensor_id);
                    break;
                }
                continue;
            }

            let read_duration = read_start.elapsed();

            let processing_start = Instant::now();
            let measurement_duration = this_data_end.duration_since(this_data_start);
            let average_dt: f32 = measurement_duration.as_secs_f32() / f32::from(fifo_level / 2);
            let dt_multiplier: f32 = average_dt * 1_000_000.0;

            // Process each pair of samples.
            let aligned_data = &fifo_data_out[0..fifo_entries];
            let iter = aligned_data.chunks_exact(2).enumerate();
            let num_chunks = iter.len();
            for (sample_index, chunk) in iter {
                let (a, b) = (chunk[0], chunk[1]);
                let (acc, gyr) = match (a.tag_sensor(), b.tag_sensor()) {
                    (TagSensor::AccelerometerNC, TagSensor::GyroscopeNC) => (a, b),
                    (TagSensor::GyroscopeNC, TagSensor::AccelerometerNC) => (b, a),
                    other => {
                        warn!("Unexpected tag in chunk: {:?}", other);
                        continue;
                    }
                };

                // This transformation transforms the sensor coordinates to the board coordinates.
                let accel_data = Acceleration::from_raw(
                    AccelerationRaw {
                        x: -acc.x(),
                        y: acc.y(),
                        z: -acc.z(),
                    },
                    self.sensor.accel_full_scale(),
                );
                let gyr_data = AngularRate::from_raw(
                    AngularRateRaw {
                        x: -gyr.x(),
                        y: gyr.y(),
                        z: -gyr.z(),
                    },
                    self.sensor.gyro_full_scale(),
                );

                let dt_micros = (dt_multiplier * sample_index as f32) as u64;
                let absolute_timestamp = this_data_start + Duration::from_micros(dt_micros);

                let _dt = match last_system_timestamp {
                    Some(last) => absolute_timestamp.duration_since(last).as_secs_f32(),
                    None => IMU_TARGET_DT,
                };

                // On the last iteration, we call the 'realtime' update function, because this is
                // the closest to the current time.
                if sample_index == num_chunks - 1 {
                    self.driver
                        .update_imu_realtime(
                            gyr_data,
                            accel_data,
                            absolute_timestamp,
                            self.sensor_id,
                        )
                        .await;
                } else {
                    self.driver
                        .update_imu_stale(gyr_data, accel_data, absolute_timestamp, self.sensor_id)
                        .await;
                }
            }

            let processing_duration = processing_start.elapsed();

            trace!(
                "{:?} FIFO read duration: {} ms",
                self.sensor_id,
                read_duration.as_secs_f32() * 1000.0
            );
            trace!(
                "{:?} Processing duration: {} ms",
                self.sensor_id,
                processing_duration.as_secs_f32() * 1000.0
            );

            // On a successful loop, reset the error counter.
            error_count = 0;
        }

        // Transition back to inactive state if too many errors occur.
        let iface = self.sensor.destroy();
        InactiveImuSensor::new(self.driver, iface, self.int, self.config, self.sensor_id)
    }
}

pub struct ImuSensorNode<'a> {
    driver: &'a InertialDriver<'a>,
    config: CommonSensorConfig,
    sensor_id: SensorId,
    int: ExtiInput<'static>,
}

impl<'a> ImuSensorNode<'a> {
    pub fn new(
        driver: &'a InertialDriver<'a>,
        config: CommonSensorConfig,
        sensor_id: SensorId,
        int: ExtiInput<'static>,
    ) -> Self {
        Self {
            driver,
            config,
            sensor_id,
            int,
        }
    }

    pub async fn run(self, iface: Interface) -> ! {
        let mut inactive =
            InactiveImuSensor::new(self.driver, iface, self.int, self.config, self.sensor_id);
        loop {
            let active = inactive.run().await;
            update_status(SensorStatus::Active, self.sensor_id);
            inactive = active.run().await;
            update_status(SensorStatus::Inactive, self.sensor_id);
        }
    }
}

#[task(pool_size = 2)]
pub async fn imu_task(
    sensor: Interface,
    driver: &'static InertialDriver<'static>,
    sensor_id: SensorId,
    int: ExtiInput<'static>,
) -> ! {
    let config = CommonSensorConfig {
        max_consecutive_errors: 5,
        base_backoff_ms: 100,
        max_backoff_ms: 20_000,
    };
    let node = ImuSensorNode::new(driver, config, sensor_id, int);
    node.run(sensor).await;

    #[allow(unreachable_code)]
    loop {
        pending::<()>().await;
    }
}
