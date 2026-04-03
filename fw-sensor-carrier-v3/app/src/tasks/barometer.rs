use defmt::{info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Timer};
use ms5607::{Ms5607, Oversampling, Uninitialized};

use super::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::resources::buses::{SharedI2c, SharedI2cBus};

const SAMPLE_INTERVAL: Duration = Duration::from_millis(25); // ~40 Hz

fn new_sensor<I2C: embedded_hal_async::i2c::I2c>(i2c: I2C) -> Ms5607<I2C, Uninitialized> {
    Ms5607::new(i2c, false)
}

async fn run_inner<I2C>(i2c: I2C) -> !
where
    I2C: embedded_hal_async::i2c::I2c,
{
    let mut uninit = new_sensor(i2c);
    let mut attempt: u8 = 0;

    loop {
        let mut sensor = loop {
            match uninit.init(&mut Delay).await {
                Ok(s) => {
                    attempt = 0;
                    break s;
                }
                Err(err) => {
                    attempt = attempt.saturating_add(1);
                    warn!("barometer: init failed (attempt {}), retrying...", attempt);
                    uninit = new_sensor(err.sensor.destroy());
                    Timer::after(backoff(attempt)).await;
                }
            }
        };

        info!("barometer: active");

        let mut errors: u8 = 0;
        loop {
            match sensor.measure(Oversampling::Osr2048, &mut Delay).await {
                Ok(m) => {
                    errors = 0;
                    trace!(
                        "barometer: pressure={} mbar, temp={} C",
                        m.pressure_mbar, m.temperature_c
                    );
                }
                Err(_) => {
                    errors = errors.saturating_add(1);
                    warn!("barometer: read error ({}/{})", errors, MAX_CONSECUTIVE_ERRORS);
                    if errors >= MAX_CONSECUTIVE_ERRORS {
                        break;
                    }
                }
            }

            Timer::after(SAMPLE_INTERVAL).await;
        }

        warn!("barometer: inactive (too many errors)");
        uninit = new_sensor(sensor.destroy());
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(bus: SharedI2cBus) -> ! {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    run_inner(i2c).await
}
