use defmt::{info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr, MagMode, MagOutputDataRate};

use crate::tasks::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::measurements::{MagData, MagSample, Timestamped};
use crate::resources::buses::{SharedI2c, SharedI2cBus};
use crate::sensors::MagnetometerId;
use crate::signals;

const SAMPLE_INTERVAL: Duration = Duration::from_millis(100); // 10 Hz, matches MagOutputDataRate::Hz10

async fn initialise<I2C: embedded_hal_async::i2c::I2c>(
    i2c: I2C,
) -> Result<Lsm303agr<lsm303agr::interface::I2cInterface<I2C>, lsm303agr::mode::MagContinuous>, I2C>
{
    let mut sensor = Lsm303agr::new_with_i2c(i2c);

    if sensor.init().await.is_err() {
        return Err(sensor.destroy());
    }

    let mut sensor = match sensor.into_mag_continuous().await {
        Ok(s) => s,
        Err(e) => return Err(e.dev.destroy()),
    };

    if sensor
        .set_mag_mode_and_odr(&mut Delay, MagMode::HighResolution, MagOutputDataRate::Hz10)
        .await
        .is_err()
    {
        return Err(sensor.destroy());
    }

    if sensor.enable_mag_offset_cancellation().await.is_err() {
        return Err(sensor.destroy());
    }

    if sensor.mag_enable_low_pass_filter().await.is_err() {
        return Err(sensor.destroy());
    }

    if sensor
        .set_accel_mode_and_odr(&mut Delay, AccelMode::Normal, AccelOutputDataRate::Hz50)
        .await
        .is_err()
    {
        return Err(sensor.destroy());
    }

    Ok(sensor)
}

struct Inactive<I2C> {
    i2c: I2C,
    id: MagnetometerId,
    delay: Duration,
    attempt: u8,
}

impl<I2C: embedded_hal_async::i2c::I2c> Inactive<I2C> {
    async fn run(
        mut self,
    ) -> Active<I2C> {
        loop {
            match initialise(self.i2c).await {
                Ok(sensor) => {
                    info!("magnetometer: active");
                    return Active {
                        sensor,
                        id: self.id,
                        delay: self.delay,
                    };
                }
                Err(i2c) => {
                    self.attempt = self.attempt.saturating_add(1);
                    warn!("magnetometer: init failed (attempt {})", self.attempt);
                    self.i2c = i2c;
                    Timer::after(backoff(self.attempt)).await;
                }
            }
        }
    }
}

struct Active<I2C> {
    sensor: Lsm303agr<lsm303agr::interface::I2cInterface<I2C>, lsm303agr::mode::MagContinuous>,
    id: MagnetometerId,
    delay: Duration,
}

impl<I2C: embedded_hal_async::i2c::I2c> Active<I2C> {
    async fn run(mut self) -> Inactive<I2C> {
        let mut errors: u8 = 0;

        loop {
            let next_sample = Instant::now() + SAMPLE_INTERVAL;

            match self.sensor.magnetic_field().await {
                Ok(field) => {
                    errors = 0;
                    let sample = MagSample {
                        sensor_id: self.id,
                        data: Timestamped::now_with_delay(
                            MagData {
                                x: field.x_raw() as i16,
                                y: field.y_raw() as i16,
                                z: field.z_raw() as i16,
                            },
                            self.delay,
                        ),
                    };
                    signals::submit_mag_sample(sample);
                    trace!("mag: x={} y={} z={}", field.x_raw(), field.y_raw(), field.z_raw());
                }
                Err(_) => {
                    errors = errors.saturating_add(1);
                    warn!("magnetometer: read error ({}/{})", errors, MAX_CONSECUTIVE_ERRORS);
                    if errors >= MAX_CONSECUTIVE_ERRORS {
                        break;
                    }
                }
            }

            Timer::at(next_sample).await;
        }

        warn!("magnetometer: inactive (too many errors)");
        Inactive {
            i2c: self.sensor.destroy(),
            id: self.id,
            delay: self.delay,
            attempt: 0,
        }
    }
}

async fn run_inner<I2C: embedded_hal_async::i2c::I2c>(
    i2c: I2C,
    id: MagnetometerId,
    delay: Duration,
) -> ! {
    let mut inactive = Inactive {
        i2c,
        id,
        delay,
        attempt: 0,
    };

    loop {
        let active = inactive.run().await;
        inactive = active.run().await;
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(bus: SharedI2cBus, id: MagnetometerId, delay: Duration) -> ! {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    run_inner(i2c, id, delay).await
}
