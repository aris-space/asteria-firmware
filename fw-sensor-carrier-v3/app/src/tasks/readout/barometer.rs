use defmt::{Debug2Format, debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use ms5607::{Ms5607, Oversampling};

use core::sync::atomic::Ordering;

use super::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::resources::buses::{SharedI2c, SharedI2cBus};
use crate::sensors::{BAROMETER_STATUS, BarometerId, SensorStatus};
use crate::signals;
use crate::types::BaroSample;

pub const SAMPLE_HZ: u32 = 40;
const SAMPLE_INTERVAL: Duration = Duration::from_millis(1000 / SAMPLE_HZ as u64);

struct Inactive<I2C> {
    sensor: Ms5607<I2C, ms5607::Uninitialized>,
    id: BarometerId,
    delay: Duration,
    attempt: u8,
}

impl<I2C: embedded_hal_async::i2c::I2c> Inactive<I2C> {
    async fn run(mut self) -> Active<I2C> {
        loop {
            debug!("{} initializing", self.id);
            match self.sensor.init(&mut Delay).await {
                Ok(sensor) => {
                    info!("{} initialized", self.id);
                    return Active {
                        sensor,
                        id: self.id,
                        delay: self.delay,
                    };
                }
                Err(err) => {
                    error!("{} init failed: {:?}", self.id, Debug2Format(&err.kind));
                    self.attempt = self.attempt.saturating_add(1);
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
                    let sample = BaroSample {
                        src: self.id,
                        ts: Instant::now() - self.delay,
                        pressure_mbar: m.pressure_mbar,
                        temperature_c: m.temperature_c,
                    };
                    signals::submit_baro_sample(sample);
                    trace!("{} p={} mbar", self.id, m.pressure_mbar);
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
        BAROMETER_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
        inactive = active.run().await;
        BAROMETER_STATUS[id.index()].store(SensorStatus::Inactive, Ordering::Relaxed);
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(bus: SharedI2cBus, id: BarometerId, delay: Duration) -> ! {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    run_inner(i2c, id, delay).await
}
