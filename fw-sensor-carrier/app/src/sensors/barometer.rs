use crate::Debug2Format;
use crate::drivers::environmental::EnvironmentalDriver;
use crate::drivers::pressure::PressureDriver;
use crate::sensors::{CommonSensorConfig, SensorId, SensorStatus, update_status};
use crate::util::ExponentialBackoff;
use core::fmt::Debug;
use core::future::pending;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::{i2c::I2c, mode::Async};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_utils::fmt::warn;
use embedded_utils::fmt::*;
use embedded_utils::{_warn, debug};
use ms5607::{Initialized, Ms5607, Oversampling};

pub const SAMPLE_FREQUENCY_HZ: u32 = 40;
const SAMPLE_INTERVAL: Duration =
    Duration::from_millis((1000f32 / SAMPLE_FREQUENCY_HZ as f32) as u64);

/// Represents the sensor when it is not yet initialized or has been reset.
///
/// This struct handles the initialization process of the sensor. On success, it
/// transitions to an active sensor state.
struct InactivePressureSensor<'a, I2C> {
    pressure_driver: &'a PressureDriver<'a>,
    environmental_driver: &'a EnvironmentalDriver<'a>,
    sensor: Ms5607<I2C, ms5607::Uninitialized>,
    attempt_count: u8,
    config: CommonSensorConfig,
    sensor_id: SensorId,
}

impl<'a, I2C, E> InactivePressureSensor<'a, I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: Debug,
{
    /// Creates a new inactive sensor instance.
    fn new(
        pressure_driver: &'a PressureDriver<'a>,
        environmental_driver: &'a EnvironmentalDriver<'a>,
        sensor: Ms5607<I2C, ms5607::Uninitialized>,
        config: CommonSensorConfig,
        barometer_id: SensorId,
    ) -> Self {
        Self {
            pressure_driver,
            environmental_driver,
            sensor,
            config,
            attempt_count: 0,
            sensor_id: barometer_id,
        }
    }

    /// Attempts to initialize the sensor.
    ///
    /// If initialization succeeds, transitions to an active sensor state.
    /// On failure, waits using an exponential backoff strategy and retries.
    async fn run(mut self) -> ActivePressureSensor<'a, I2C> {
        loop {
            debug!("{:?} Sensor initializing", self.sensor_id);
            match self.sensor.init(&mut Delay).await {
                Ok(initialized_sensor) => {
                    // Reset the attempt counter on successful initialization.
                    self.attempt_count = 0;
                    info!("{:?} Sensor initialized", self.sensor_id);
                    return ActivePressureSensor::new(
                        self.pressure_driver,
                        self.environmental_driver,
                        initialized_sensor,
                        self.config,
                        self.sensor_id,
                    );
                }
                Err(err) => {
                    //error!("{:?} Initialization failed,", self.sensor_id);
                    error!(
                        "{:?} Initialization error: {:?}",
                        self.sensor_id,
                        Debug2Format(&err)
                    );
                    // If initialization fails, increment the attempt counter and wait.
                    self.attempt_count += 1;
                    // Wait with exponential backoff based on the number of attempts.
                    let backoff = ExponentialBackoff::new(
                        self.config.base_backoff_ms,
                        self.config.max_backoff_ms,
                    );
                    backoff.wait(self.attempt_count).await;
                    // Recreate the sensor instance using the same I2C address.
                    let old_addr = err.sensor.address();
                    let i2c = err.sensor.destroy();
                    self.sensor = Ms5607::new_with_addr(i2c, old_addr);
                }
            }
        }
    }

    /*todo can we make the top thingy nicer by adding this?
    // Backoff + recreate
    async fn handle_retry(&mut self, err: InitError<I2C, E>) {
        self.attempt_count += 1;
        let backoff = ExponentialBackoff::new(
            self.config.base_backoff_ms,
            self.config.max_backoff_ms,
        );
        backoff.wait(self.attempt_count).await;
        let old_addr = err.sensor.address();
        let i2c = err.sensor.destroy();
        self.sensor = Ms5607::new_with_addr(i2c, old_addr);
    }
     */
}

/// Represents the sensor when it is actively taking measurements.
///
/// This struct continuously reads pressure data and updates the driver. If too many
/// errors occur during measurement, it transitions back to an inactive state.
struct ActivePressureSensor<'a, I2C> {
    pressure_driver: &'a PressureDriver<'a>,
    environmental_driver: &'a EnvironmentalDriver<'a>,
    sensor: Ms5607<I2C, Initialized>,
    config: CommonSensorConfig,
    error_count: u8,
    sensor_id: SensorId,
}

impl<'a, I2C, E> ActivePressureSensor<'a, I2C>
where
    I2C: embedded_hal_async::i2c::I2c<Error = E>,
    E: Debug,
{
    /// Creates a new active sensor instance.
    fn new(
        pressure_driver: &'a PressureDriver<'a>,
        environmental_driver: &'a EnvironmentalDriver<'a>,
        sensor: Ms5607<I2C, Initialized>,
        config: CommonSensorConfig,
        barometer_id: SensorId,
    ) -> Self {
        Self {
            pressure_driver,
            environmental_driver,
            sensor,
            config,
            error_count: 0,
            sensor_id: barometer_id,
        }
    }

    /// Continuously measures pressure.
    ///
    /// On each successful measurement, the sensor's error counter is reset and the
    /// driver is updated with the new pressure data. If the number of consecutive errors
    /// exceeds the configured limit, the sensor transitions back to the inactive state.
    async fn run(mut self) -> InactivePressureSensor<'a, I2C> {
        loop {
            let next_sample = Instant::now() + SAMPLE_INTERVAL;

            match self.sensor.measure(Oversampling::Osr2048, &mut Delay).await {
                Ok(measurement) => {
                    let ts = Instant::now(); // this approximates the sample time

                    self.error_count = 0;
                    self.pressure_driver
                        .update_with_raw_data(measurement.pressure_mbar)
                        .await;

                    self.environmental_driver
                        .update_pressure_data(measurement.pressure_mbar, ts)
                        .await;
                    // Measurement succeeded; continue looping.
                }
                Err(err) => {
                    self.error_count += 1;
                    _warn!("{:?} Read error: {:?}", self.sensor_id, Debug2Format(&err));
                    if self.error_count >= self.config.max_consecutive_errors {
                        break;
                    }
                }
            }

            if Instant::now() > next_sample {
                warn!(
                    "{:?} cannot keep up with measurement interval. Took {:?}ms too long",
                    self.sensor_id,
                    (Instant::now() - next_sample).as_millis()
                );
            } else {
                Timer::at(next_sample).await;
            }
        }

        error!("{:?} Sensor offline (too many errors)", self.sensor_id);

        // Get back the I2C interface from the sensor
        let old_addr = self.sensor.address();
        let i2c = self.sensor.destroy();
        let sensor = Ms5607::new_with_addr(i2c, old_addr);
        InactivePressureSensor::new(
            self.pressure_driver,
            self.environmental_driver,
            sensor,
            self.config,
            self.sensor_id,
        )
    }
}

/// Orchestrates the sensor lifecycle by alternating between inactive (initialization)
/// and active (measurement) states.
struct PressureSensorNode<'a> {
    pressure_driver: &'a PressureDriver<'a>,
    environmental_driver: &'a EnvironmentalDriver<'a>,
    config: CommonSensorConfig,
    sensor_id: SensorId,
}

impl<'a> PressureSensorNode<'a> {
    /// Creates a new sensor node.
    fn new(
        pressure_driver: &'a PressureDriver<'a>,
        environmental_driver: &'a EnvironmentalDriver<'a>,
        config: CommonSensorConfig,
        barometer_id: SensorId,
    ) -> Self {
        Self {
            pressure_driver,
            environmental_driver,
            config,
            sensor_id: barometer_id,
        }
    }

    /// Runs the sensor node indefinitely by transitioning between inactive and active states.
    ///
    /// The sensor is first initialized (inactive state). Once active, it continuously measures
    /// pressure data until errors force it to reset. This cycle repeats indefinitely.
    pub async fn run<I2C, E>(self, initial_sensor: Ms5607<I2C, ms5607::Uninitialized>) -> !
    where
        I2C: embedded_hal_async::i2c::I2c<Error = E>,
        E: Debug,
    {
        let mut inactive = InactivePressureSensor::new(
            self.pressure_driver,
            self.environmental_driver,
            initial_sensor,
            self.config,
            self.sensor_id,
        );

        loop {
            let active = inactive.run().await;
            update_status(SensorStatus::Active, active.sensor_id);
            inactive = active.run().await;
            update_status(SensorStatus::Inactive, inactive.sensor_id);
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn barometer_task(
    ms5607: Ms5607<
        I2cDevice<'static, ThreadModeRawMutex, I2c<'static, Async, embassy_stm32::i2c::mode::Master>>,
        ms5607::Uninitialized,
    >,
    pressure_driver: &'static PressureDriver<'static>,
    environmental_driver: &'static EnvironmentalDriver<'static>,
    sensor_id: SensorId,
) -> ! {
    let config = CommonSensorConfig {
        max_consecutive_errors: 5,
        base_backoff_ms: 100,
        max_backoff_ms: 20_000,
    };

    let node = PressureSensorNode::new(pressure_driver, environmental_driver, config, sensor_id);
    node.run(ms5607).await;

    // This loop is unreachable, but is included to satisfy type-checking.
    #[allow(unreachable_code)]
    loop {
        pending::<()>().await;
    }
}
