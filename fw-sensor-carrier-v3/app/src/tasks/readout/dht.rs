use core::sync::atomic::Ordering;

use defmt::{info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use sht4x::{Precision, Sht4xAsync};

use crate::measurements::{EnvData, EnvSample, Timestamped};
use crate::resources::buses::{SharedI2c, SharedI2cBus};
use crate::sensors::{DHT_STATUS, DhtId, SensorStatus};
use crate::signals;
use crate::tasks::{MAX_CONSECUTIVE_ERRORS, backoff};

const SAMPLE_INTERVAL: Duration = Duration::from_millis(1000); // 1 Hz

struct Inactive<I2C> {
    sensor: Sht4xAsync<I2C, Delay>,
    id: DhtId,
    delay: Duration,
    attempt: u8,
}

impl<I2C: embedded_hal_async::i2c::I2c> Inactive<I2C> {
    async fn run(mut self) -> Active<I2C> {
        loop {
            if self.sensor.soft_reset(&mut Delay).await.is_err() {
                self.attempt = self.attempt.saturating_add(1);
                warn!("dht: soft reset failed (attempt {})", self.attempt);
                Timer::after(backoff(self.attempt)).await;
                continue;
            }

            if self
                .sensor
                .measure(Precision::Low, &mut Delay)
                .await
                .is_err()
            {
                self.attempt = self.attempt.saturating_add(1);
                warn!("dht: first measurement failed (attempt {})", self.attempt);
                Timer::after(backoff(self.attempt)).await;
                continue;
            }

            info!("dht: active");
            DHT_STATUS[self.id.index()].store(SensorStatus::Active, Ordering::Relaxed);
            return Active {
                sensor: self.sensor,
                id: self.id,
                delay: self.delay,
            };
        }
    }
}

struct Active<I2C> {
    sensor: Sht4xAsync<I2C, Delay>,
    id: DhtId,
    delay: Duration,
}

impl<I2C: embedded_hal_async::i2c::I2c> Active<I2C> {
    async fn run(mut self) -> Inactive<I2C> {
        let mut errors: u8 = 0;

        loop {
            let next_sample = Instant::now() + SAMPLE_INTERVAL;

            match self.sensor.measure(Precision::Low, &mut Delay).await {
                Ok(m) => {
                    errors = 0;
                    let temperature_c: f32 = m.temperature_celsius().to_num();
                    let humidity_rh: f32 = m.humidity_percent().to_num();
                    let sample = EnvSample {
                        sensor_id: self.id,
                        data: Timestamped::now_with_delay(
                            EnvData {
                                temperature_c,
                                humidity_rh,
                            },
                            self.delay,
                        ),
                    };
                    signals::submit_env_sample(sample);
                    trace!("dht: t={} c, rh={} %", temperature_c, humidity_rh);
                }
                Err(_) => {
                    errors = errors.saturating_add(1);
                    warn!("dht: read error ({}/{})", errors, MAX_CONSECUTIVE_ERRORS);
                    if errors >= MAX_CONSECUTIVE_ERRORS {
                        break;
                    }
                }
            }

            if Instant::now() <= next_sample {
                Timer::at(next_sample).await;
            }
        }

        warn!("dht: inactive (too many errors)");
        DHT_STATUS[self.id.index()].store(SensorStatus::Inactive, Ordering::Relaxed);
        Inactive {
            sensor: Sht4xAsync::new(self.sensor.destroy()),
            id: self.id,
            delay: self.delay,
            attempt: 0,
        }
    }
}

async fn run_inner<I2C>(i2c: I2C, id: DhtId, delay: Duration) -> !
where
    I2C: embedded_hal_async::i2c::I2c,
{
    let mut inactive = Inactive {
        sensor: Sht4xAsync::new(i2c),
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
pub async fn task(bus: SharedI2cBus, id: DhtId, delay: Duration) -> ! {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    run_inner(i2c, id, delay).await
}
