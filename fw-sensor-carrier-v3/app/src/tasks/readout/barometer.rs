use defmt::{info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use ms5607::{Ms5607, Oversampling};

use crate::tasks::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::measurements::{PressureData, PressureSample, Timestamped};
use crate::resources::buses::{SharedI2c, SharedI2cBus};
use crate::sensors::BarometerId;
use crate::signals;

const SAMPLE_INTERVAL: Duration = Duration::from_millis(25); // ~40 Hz

struct Inactive<I2C> {
    sensor: Ms5607<I2C, ms5607::Uninitialized>,
    id: BarometerId,
    delay: Duration,
    attempt: u8,
}

impl<I2C: embedded_hal_async::i2c::I2c> Inactive<I2C> {
    async fn run(mut self) -> Active<I2C> {
        loop {
            match self.sensor.init(&mut Delay).await {
                Ok(sensor) => {
                    info!("barometer: active");
                    return Active {
                        sensor,
                        id: self.id,
                        delay: self.delay,
                    };
                }
                Err(err) => {
                    self.attempt = self.attempt.saturating_add(1);
                    warn!("barometer: init failed (attempt {})", self.attempt);
                    self.sensor = Ms5607::new(err.sensor.destroy(), false);
                    Timer::after(backoff(self.attempt)).await;
                }
            }
        }
    }
}

struct Active<I2C> {
    sensor: Ms5607<I2C, ms5607::Initialized>,
    id: BarometerId,
    delay: Duration,
}

impl<I2C: embedded_hal_async::i2c::I2c> Active<I2C> {
    async fn run(mut self) -> Inactive<I2C> {
        let mut errors: u8 = 0;

        loop {
            let next_sample = Instant::now() + SAMPLE_INTERVAL;

            match self.sensor.measure(Oversampling::Osr2048, &mut Delay).await {
                Ok(m) => {
                    errors = 0;
                    let sample = PressureSample {
                        sensor_id: self.id,
                        data: Timestamped::now_with_delay(
                            PressureData {
                                pressure_mbar: m.pressure_mbar,
                                temperature_c: m.temperature_c,
                            },
                            self.delay,
                        ),
                    };
                    signals::submit_pressure_sample(sample);
                    trace!("barometer: p={} mbar", m.pressure_mbar);
                }
                Err(_) => {
                    errors = errors.saturating_add(1);
                    warn!("barometer: read error ({}/{})", errors, MAX_CONSECUTIVE_ERRORS);
                    if errors >= MAX_CONSECUTIVE_ERRORS {
                        break;
                    }
                }
            }

            Timer::at(next_sample).await;
        }

        warn!("barometer: inactive (too many errors)");
        Inactive {
            sensor: Ms5607::new(self.sensor.destroy(), false),
            id: self.id,
            delay: self.delay,
            attempt: 0,
        }
    }
}

async fn run_inner<I2C>(i2c: I2C, id: BarometerId, delay: Duration) -> !
where
    I2C: embedded_hal_async::i2c::I2c,
{
    let mut inactive = Inactive {
        sensor: Ms5607::new(i2c, false),
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
pub async fn task(bus: SharedI2cBus, id: BarometerId, delay: Duration) -> ! {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    run_inner(i2c, id, delay).await
}
