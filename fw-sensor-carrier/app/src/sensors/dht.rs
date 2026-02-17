use crate::Debug2Format;
use crate::drivers::environmental::EnvironmentalDriver;
use crate::sensors::{CommonSensorConfig, SensorId, SensorStatus, update_status};
use crate::util::ExponentialBackoff;
use core::fmt::Debug;
use core::future::pending;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::{i2c::I2c, mode::Async};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_utils::fmt::*;
use sht4x::{Precision, Sht4xAsync};

pub const SAMPLE_FREQUENCY_HZ: u32 = 1;
const SAMPLE_INTERVAL: Duration =
    Duration::from_millis((1000f32 / SAMPLE_FREQUENCY_HZ as f32) as u64);

struct InactiveDhtSensor<'a, I2C> {
    driver: &'a EnvironmentalDriver<'a>,
    sensor: Sht4xAsync<I2C, Delay>,
    config: CommonSensorConfig,
    sensor_id: SensorId,
    attempt_count: u8,
}

impl<'a, I2C, E> InactiveDhtSensor<'a, I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: Debug,
{
    fn new(
        driver: &'a EnvironmentalDriver<'a>,
        sensor: Sht4xAsync<I2C, Delay>,
        config: CommonSensorConfig,
        sensor_id: SensorId,
    ) -> Self {
        Self {
            driver,
            sensor,
            config,
            sensor_id,
            attempt_count: 0,
        }
    }

    async fn run(mut self) -> ActiveDhtSensor<'a, I2C> {
        loop {
            debug!("{:?} DHT initializing", self.sensor_id);

            let reset_ok = match self.sensor.soft_reset(&mut Delay).await {
                Ok(()) => true,
                Err(err) => {
                    error!(
                        "{:?} Soft reset failed: {:?}",
                        self.sensor_id,
                        Debug2Format(&err)
                    );
                    false
                }
            };

            if reset_ok {
                match self.sensor.measure(Precision::Low, &mut Delay).await {
                    Ok(_) => {
                        self.attempt_count = 0;
                        info!("{:?} DHT initialized", self.sensor_id);
                        return ActiveDhtSensor::new(
                            self.driver,
                            self.sensor,
                            self.config,
                            self.sensor_id,
                        );
                    }
                    Err(err) => {
                        error!(
                            "{:?} First measurement failed: {:?}",
                            self.sensor_id,
                            Debug2Format(&err)
                        );
                    }
                }
            }

            self.attempt_count += 1;
            let backoff =
                ExponentialBackoff::new(self.config.base_backoff_ms, self.config.max_backoff_ms);
            backoff.wait(self.attempt_count).await;
        }
    }
}

struct ActiveDhtSensor<'a, I2C> {
    driver: &'a EnvironmentalDriver<'a>,
    sensor: Sht4xAsync<I2C, Delay>,
    config: CommonSensorConfig,
    sensor_id: SensorId,
    error_count: u8,
}

impl<'a, I2C, E> ActiveDhtSensor<'a, I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: Debug,
{
    fn new(
        driver: &'a EnvironmentalDriver<'a>,
        sensor: Sht4xAsync<I2C, Delay>,
        config: CommonSensorConfig,
        sensor_id: SensorId,
    ) -> Self {
        Self {
            driver,
            sensor,
            config,
            sensor_id,
            error_count: 0,
        }
    }

    async fn run(mut self) -> InactiveDhtSensor<'a, I2C> {
        loop {
            let next_sample = Instant::now() + SAMPLE_INTERVAL;
            match self.sensor.measure(Precision::Low, &mut Delay).await {
                Ok(meas) => {
                    let ts = Instant::now(); // this approximates the sample time
                    self.error_count = 0;
                    self.driver
                        .update_temp_hum_data(
                            meas.temperature_celsius().to_num(),
                            meas.humidity_percent().to_num(),
                            ts,
                        )
                        .await;
                }
                Err(err) => {
                    error!(
                        "{:?} Measurement failed: {:?}",
                        self.sensor_id,
                        Debug2Format(&err)
                    );

                    self.error_count += 1;
                    warn!("{:?} Measurement failed", self.sensor_id);
                    if self.error_count >= self.config.max_consecutive_errors {
                        error!("{:?} DHT offline", self.sensor_id);
                        let i2c = self.sensor.destroy();
                        let sensor = Sht4xAsync::new(i2c);
                        return InactiveDhtSensor::new(
                            self.driver,
                            sensor,
                            self.config,
                            self.sensor_id,
                        );
                    }
                }
            }

            if Instant::now() > next_sample {
                warn!(
                    "{:?} cannot keep up with measurement interval.",
                    self.sensor_id
                );
            } else {
                Timer::at(next_sample).await;
            }
        }
    }
}

struct DhtSensorNode<'a> {
    driver: &'a EnvironmentalDriver<'a>,
    config: CommonSensorConfig,
    sensor_id: SensorId,
}

impl<'a> DhtSensorNode<'a> {
    fn new(
        driver: &'static EnvironmentalDriver<'a>,
        config: CommonSensorConfig,
        sensor_id: SensorId,
    ) -> Self {
        Self {
            driver,
            config,
            sensor_id,
        }
    }

    pub async fn run<I2C, E>(self, initial_sensor: Sht4xAsync<I2C, Delay>) -> !
    where
        I2C: embedded_hal_async::i2c::I2c<Error = E>,
        E: Debug,
    {
        let mut inactive =
            InactiveDhtSensor::new(self.driver, initial_sensor, self.config, self.sensor_id);

        loop {
            let active = inactive.run().await;
            update_status(SensorStatus::Active, self.sensor_id);
            inactive = active.run().await;
            update_status(SensorStatus::Inactive, self.sensor_id);
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn dht_task(
    sht4x: Sht4xAsync<I2cDevice<'static, ThreadModeRawMutex, I2c<'static, Async>>, Delay>,
    driver: &'static EnvironmentalDriver<'static>,
    sensor_id: SensorId,
) -> ! {
    let config = CommonSensorConfig {
        max_consecutive_errors: 5,
        base_backoff_ms: 100,
        max_backoff_ms: 20_000,
    };

    let node = DhtSensorNode::new(driver, config, sensor_id);
    node.run(sht4x).await;

    #[allow(unreachable_code)]
    loop {
        pending::<()>().await;
    }
}
