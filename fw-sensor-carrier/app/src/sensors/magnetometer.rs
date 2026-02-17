use crate::drivers::magnetic_field::{MagMeasurement, MagneticField, MagneticFieldDriver};
use crate::sensors::{CommonSensorConfig, SensorId, SensorStatus, update_status};
use crate::util::ExponentialBackoff;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_utils::fmt::*;
use lsm303agr::interface::I2cInterface;
use lsm303agr::mode::MagContinuous;
use lsm303agr::{Lsm303agr, MagMode, MagOutputDataRate};

#[cfg(feature = "defmt")]
use defmt::Debug2Format;
use embassy_sync::pubsub::PubSubChannel;

#[cfg(not(feature = "defmt"))]
struct Debug2Format<'a, T: core::fmt::Debug + ?Sized>(pub &'a T);

const SAMPLE_FREQUENCY_HZ: u32 = 100;
const SAMPLE_INTERVAL: Duration =
    Duration::from_millis((1000f32 / SAMPLE_FREQUENCY_HZ as f32) as u64);

#[allow(dead_code)]
pub static MAGNETOMETER_1_RAW: PubSubChannel<ThreadModeRawMutex, MagMeasurement, 10, 1, 1> =
    PubSubChannel::new();
#[allow(dead_code)]
pub static MAGNETOMETER_2_RAW: PubSubChannel<ThreadModeRawMutex, MagMeasurement, 10, 1, 1> =
    PubSubChannel::new();

/// Inactive state of the magnetometer sensor.
/// The sensor owns its I2C interface.
struct InactiveMagSensor<'a> {
    driver: &'a MagneticFieldDriver<'a>,
    interface: Interface,
    config: CommonSensorConfig,
    sensor_id: SensorId,
    attempt_count: u8,
}

impl<'a> InactiveMagSensor<'a> {
    fn new(
        driver: &'a MagneticFieldDriver<'a>,
        interface: Interface,
        config: CommonSensorConfig,
        sensor_id: SensorId,
    ) -> Self {
        Self {
            driver,
            interface,
            config,
            sensor_id,
            attempt_count: 0,
        }
    }

    /// Attempt to initialize the sensor, retrying on failure.
    async fn run(mut self) -> ActiveMagSensor<'a> {
        loop {
            debug!("{:?} Magnetometer initializing", self.sensor_id);
            match initialise(self.interface).await {
                Ok(sensor) => {
                    info!("{:?} Magnetometer initialized", self.sensor_id);
                    return ActiveMagSensor::new(self.driver, sensor, self.config, self.sensor_id);
                }
                Err(iface) => {
                    error!("{:?} Magnetometer initialization failed", self.sensor_id);

                    // Recover the interface so we may retry.
                    self.interface = iface;
                    self.attempt_count += 1;
                    let backoff = ExponentialBackoff::new(
                        self.config.base_backoff_ms,
                        self.config.max_backoff_ms,
                    );
                    backoff.wait(self.attempt_count).await;
                }
            }
        }
    }
}

/// Initialise the sensor using its I2C interface.
/// On success, returns a configured sensor. On failure, returns the error along with the I2C interface.
/// This avoids using a mutable reference by transferring ownership.
async fn initialise(
    interface: Interface,
) -> Result<Lsm303agr<I2cInterface<Interface>, MagContinuous>, Interface> {
    // Create a sensor instance by consuming the I2C bus.
    let mut sensor = Lsm303agr::new_with_i2c(interface);

    // Initialize the sensor.
    if let Err(e) = sensor.init().await {
        error!("Failed to initialize magnetometer: {:?}", Debug2Format(&e));
        return Err(sensor.destroy());
    }

    // Switch to continuous magnetometer mode.
    let mut sensor = match sensor.into_mag_continuous().await {
        Ok(s) => s,
        Err(e) => {
            error!(
                "Failed to initialize magnetometer: {:?}",
                Debug2Format(&e.error)
            );
            let err = e.dev.destroy();
            return Err(err);
        }
    };

    // Set the magnetometer mode and output data rate.
    if let Err(e) = sensor
        .set_mag_mode_and_odr(&mut Delay, MagMode::HighResolution, MagOutputDataRate::Hz10)
        .await
    {
        error!("Failed to initialize magnetometer: {:?}", Debug2Format(&e));
        return Err(sensor.destroy());
    }

    // Enable offset cancellation.
    if let Err(e) = sensor.enable_mag_offset_cancellation().await {
        error!("Failed to initialize magnetometer: {:?}", Debug2Format(&e));
        return Err(sensor.destroy());
    }

    // Enable low pass filtering.
    if let Err(e) = sensor.mag_enable_low_pass_filter().await {
        error!("Failed to initialize magnetometer: {:?}", Debug2Format(&e));
        return Err(sensor.destroy());
    }

    Ok(sensor)
}

/// Active state of the magnetometer sensor.
struct ActiveMagSensor<'a> {
    driver: &'a MagneticFieldDriver<'a>,
    sensor: Lsm303agr<I2cInterface<Interface>, MagContinuous>,
    config: CommonSensorConfig,
    sensor_id: SensorId,
    error_count: u8,
}

impl<'a> ActiveMagSensor<'a> {
    fn new(
        driver: &'a MagneticFieldDriver<'a>,
        sensor: Lsm303agr<I2cInterface<Interface>, MagContinuous>,
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

    async fn run(mut self) -> InactiveMagSensor<'a> {
        loop {
            let next_sample = Instant::now() + SAMPLE_INTERVAL;
            match self.sensor.magnetic_field().await {
                Ok(mag_field) => {
                    let now = Instant::now();

                    // this transformation maps the sensor's coordinate system to the
                    // coordinate system of our board.
                    let mag_field = MagneticField::new(
                        mag_field.x_raw().wrapping_neg(),
                        mag_field.y_raw().wrapping_neg(),
                        mag_field.z_raw().wrapping_neg(),
                    );

                    // Hard iron bias
                    let (hard_iron_bias, soft_iron_matrix) = mag_calibration_for(self.sensor_id);
                    let vector = nalgebra::Vector3::new(
                        mag_field.x_nt() as f32,
                        mag_field.y_nt() as f32,
                        mag_field.z_nt() as f32,
                    );

                    let calibrated = soft_iron_matrix * (vector - hard_iron_bias);

                    let mag_calibrated = MagneticField::new(
                        calibrated.x as i16 as u16,
                        calibrated.y as i16 as u16,
                        calibrated.z as i16 as u16,
                    );
                    let measurement = (now, mag_calibrated);

                    match self.sensor_id {
                        SensorId::MagnetometerBus1 => MAGNETOMETER_1_RAW
                            .immediate_publisher()
                            .publish_immediate(measurement),
                        SensorId::MagnetometerBus2 => MAGNETOMETER_2_RAW
                            .immediate_publisher()
                            .publish_immediate(measurement),
                        _ => embedded_utils::unreachable!("Not possible."),
                    }

                    // Forward the measurement with its timestamp.
                    self.driver
                        .update_magnetometer(measurement, self.sensor_id)
                        .await;
                }
                Err(err) => {
                    warn!(
                        "{:?} Measurement failed with error: {:?}",
                        self.sensor_id,
                        Debug2Format(&err)
                    );
                    self.error_count += 1;
                    if self.error_count >= self.config.max_consecutive_errors {
                        break;
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
        error!("{:?} Magnetometer offline", self.sensor_id);

        // Get back the I2C interface from the sensor.
        let iface = self.sensor.destroy();
        InactiveMagSensor::new(self.driver, iface, self.config, self.sensor_id)
    }
}

/// Node representing the magnetometer sensor.
struct MagSensorNode<'a> {
    driver: &'a MagneticFieldDriver<'a>,
    config: CommonSensorConfig,
    sensor_id: SensorId,
}

impl<'a> MagSensorNode<'a> {
    fn new(
        driver: &'a MagneticFieldDriver<'a>,
        config: CommonSensorConfig,
        sensor_id: SensorId,
    ) -> Self {
        Self {
            driver,
            config,
            sensor_id,
        }
    }

    /// Run the sensor node continuously.
    pub async fn run(self, interface: Interface) -> ! {
        let mut inactive =
            InactiveMagSensor::new(self.driver, interface, self.config, self.sensor_id);
        loop {
            let active = inactive.run().await;
            update_status(SensorStatus::Active, self.sensor_id);
            inactive = active.run().await;
            update_status(SensorStatus::Inactive, self.sensor_id);
        }
    }
}

#[allow(clippy::excessive_precision)]
fn mag_calibration_for(sensor: SensorId) -> (nalgebra::Vector3<f32>, nalgebra::Matrix3<f32>) {
    match sensor {
        SensorId::MagnetometerBus1 => {
            // Magnetometer 1 (nT)
            let hard_iron_bias = nalgebra::Vector3::new(
                -5548.220872821729_f32,
                3827.5558491680104_f32,
                1103.1746334467373_f32,
            );
            let soft_iron_matrix = nalgebra::Matrix3::new(
                0.9933533874801035_f32,
                0.02660683424766733_f32,
                -0.012040160633120614_f32,
                0.0266068342476673_f32,
                1.0749237129791611_f32,
                -0.004425871227641531_f32,
                -0.01204016063312053_f32,
                -0.0044258712276415224_f32,
                0.9896712863098899_f32,
            );
            (hard_iron_bias, soft_iron_matrix)
        }
        SensorId::MagnetometerBus2 => {
            // Magnetometer 2 (nT)
            let hard_iron_bias = nalgebra::Vector3::new(
                1877.0587380839947_f32,
                7553.287522291296_f32,
                6455.773615317315_f32,
            );
            let soft_iron_matrix = nalgebra::Matrix3::new(
                0.9947437765775108_f32,
                0.02142849359925576_f32,
                0.013753388737255394_f32,
                0.021428493599255673_f32,
                1.056768106384783_f32,
                -0.003894936674428707_f32,
                0.013753388737255299_f32,
                -0.003894936674428721_f32,
                0.999343144534608_f32,
            );
            (hard_iron_bias, soft_iron_matrix)
        }
        _ => embedded_utils::unreachable!("Not possible."),
    }
}

type Interface = I2cDevice<'static, ThreadModeRawMutex, I2c<'static, Async>>;

#[embassy_executor::task(pool_size = 2)]
pub async fn magnetometer_task(
    iface: Interface,
    driver: &'static MagneticFieldDriver<'static>,
    sensor_id: SensorId,
) -> ! {
    let config = CommonSensorConfig {
        max_consecutive_errors: 5,
        base_backoff_ms: 100,
        max_backoff_ms: 20_000,
    };
    let node = MagSensorNode::new(driver, config, sensor_id);
    node.run(iface).await;
    #[allow(unreachable_code)]
    loop {
        core::future::pending::<()>().await;
    }
}
