use defmt::{info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Timer};
use ms5607::{Ms5607, Oversampling};

use super::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::resources::buses::{SharedI2c, SharedI2cBus};
use crate::sensors::BarometerId;
use crate::measurements::{PressureData, PressureSample, Timestamped};
use crate::signals;

const SAMPLE_INTERVAL: Duration = Duration::from_millis(25); // ~40 Hz

async fn run_inner<I2C>(i2c: I2C, id: BarometerId) -> !
where
    I2C: embedded_hal_async::i2c::I2c,
{
    let mut uninit = Ms5607::new(i2c, false);
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
                    uninit = Ms5607::new(err.sensor.destroy(), false);
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
                    let sample = PressureSample {
                        sensor_id: id,
                        data: Timestamped::now(PressureData {
                            pressure_mbar: m.pressure_mbar,
                            temperature_c: m.temperature_c,
                        }),
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

            Timer::after(SAMPLE_INTERVAL).await;
        }

        warn!("barometer: inactive (too many errors)");
        uninit = Ms5607::new(sensor.destroy(), false);
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(bus: SharedI2cBus, id: BarometerId) -> ! {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    run_inner(i2c, id).await
}
