use core::sync::atomic::Ordering;

use defmt::{Debug2Format, debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use sht4x::{Precision, Sht4xAsync};

use super::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::measurements::{EnvData, EnvSample, Timestamped};
use crate::resources::buses::{SharedI2c, SharedI2cBus};
use crate::sensors::{DHT_STATUS, DhtId, SensorStatus};
use crate::signals;

pub const SAMPLE_HZ: u32 = 1;
const SAMPLE_INTERVAL: Duration = Duration::from_millis(1000 / SAMPLE_HZ as u64);

struct Inactive<I2C> {
    sensor: Sht4xAsync<I2C, Delay>,
    id: DhtId,
    delay: Duration,
    attempt: u8,
}

impl<I2C: embedded_hal_async::i2c::I2c> Inactive<I2C> {
    async fn run(mut self) -> Active<I2C> {
        loop {
            debug!("{} initializing", self.id);

            let result = match self.sensor.soft_reset(&mut Delay).await {
                Ok(()) => self
                    .sensor
                    .measure(Precision::Low, &mut Delay)
                    .await
                    .map(|_| ()),
                Err(e) => Err(e),
            };

            match result {
                Ok(()) => {
                    info!("{} initialized", self.id);
                    return Active {
                        sensor: self.sensor,
                        id: self.id,
                        delay: self.delay,
                    };
                }
                Err(e) => {
                    error!("{} init failed: {:?}", self.id, Debug2Format(&e));
                    self.attempt = self.attempt.saturating_add(1);
                    Timer::after(backoff(self.attempt)).await;
                }
            }
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
                    trace!("{} t={} c rh={} %", self.id, temperature_c, humidity_rh);
                }
                Err(e) => {
                    warn!("{} read error: {:?}", self.id, Debug2Format(&e));
                    errors = errors.saturating_add(1);
                    if errors >= MAX_CONSECUTIVE_ERRORS {
                        break;
                    }
                }
            }

            if Instant::now() > next_sample {
                warn!("{} can't keep up with sample interval", self.id);
            } else {
                Timer::at(next_sample).await;
            }
        }

        error!("{} offline (too many consecutive errors)", self.id);
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
        DHT_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
        inactive = active.run().await;
        DHT_STATUS[id.index()].store(SensorStatus::Inactive, Ordering::Relaxed);
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(bus: SharedI2cBus, id: DhtId, delay: Duration) -> ! {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    run_inner(i2c, id, delay).await
}
