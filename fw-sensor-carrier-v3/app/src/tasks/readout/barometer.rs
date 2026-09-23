use core::sync::atomic::Ordering;

use defmt::{Debug2Format, debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use ms5607::{Ms5607, Oversampling};

use super::{I2C_RECOVERY_INTERVAL, MAX_CONSECUTIVE_ERRORS, MAX_INIT_ATTEMPTS, backoff};
use crate::resources::buses::{self, SharedI2c, SharedI2cBus};
use crate::sensors::{BAROMETER_STATUS, BarometerId, SensorStatus};
use crate::signals;
use crate::types::BaroSample;

pub const SAMPLE_HZ: u32 = 40;
const SAMPLE_INTERVAL: Duration = Duration::from_millis(1000 / SAMPLE_HZ as u64);

type BusDevice = I2cDevice<'static, CriticalSectionRawMutex, SharedI2c>;
/// A fully-initialized barometer, ready to read.
pub type Sensor = Ms5607<BusDevice, ms5607::Initialized>;

/// Bring the barometer up, retrying a bounded number of times. Returns the live
/// sensor, or `None` (and marks it `Disabled`) if it never answered. Call this
/// sequentially at startup, before any read task runs, so healthy sensors
/// get their first initialization attempt before background retries begin.
/// The read task retries a failed initialization.
pub async fn init(bus: SharedI2cBus, id: BarometerId) -> Option<Sensor> {
    for attempt in 1..=MAX_INIT_ATTEMPTS {
        debug!("{} initializing (attempt {})", id, attempt);
        let sensor = Ms5607::new(I2cDevice::new(bus), false);
        match sensor.init(&mut Delay).await {
            Ok(sensor) => {
                info!("{} initialized", id);
                return Some(sensor);
            }
            Err(err) => {
                error!("{} init failed: {:?}", id, Debug2Format(&err.kind));
                if attempt < MAX_INIT_ATTEMPTS {
                    Timer::after(backoff(attempt)).await;
                }
            }
        }
    }
    warn!("{} not detected; retrying later", id);
    BAROMETER_STATUS[id.index()].store(SensorStatus::Disabled, Ordering::Relaxed);
    None
}

#[embassy_executor::task(pool_size = 2)]
pub async fn read_task(mut sensor: Option<Sensor>, bus: SharedI2cBus, id: BarometerId) -> ! {
    while sensor.is_none() {
        Timer::after(I2C_RECOVERY_INTERVAL).await;
        buses::recover(bus).await;
        sensor = init(bus, id).await;
    }
    let mut sensor = sensor.expect("barometer initialized before reading");
    BAROMETER_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
    let mut errors: u8 = 0;

    loop {
        let measurement_started = Instant::now();
        let next_sample = measurement_started + SAMPLE_INTERVAL;

        match sensor.measure(Oversampling::Osr2048, &mut Delay).await {
            Ok(m) => {
                if errors >= MAX_CONSECUTIVE_ERRORS {
                    info!("{} recovered", id);
                    BAROMETER_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
                }
                errors = 0;
                // Pressure is converted in the first half of the D1/D2 cycle.
                // Use the measured cycle midpoint instead of a fixed read delay.
                let measurement_duration_us = Instant::now()
                    .saturating_duration_since(measurement_started)
                    .as_micros();
                let sample = BaroSample {
                    src: id,
                    ts: measurement_started + Duration::from_micros(measurement_duration_us / 2),
                    pressure_mbar: m.pressure_mbar,
                    temperature_c: m.temperature_c,
                };
                signals::submit_baro_sample(sample);
                trace!("{} p={} mbar", id, m.pressure_mbar);
            }
            Err(e) => {
                warn!("{} read error: {:?}", id, Debug2Format(&e));
                errors = errors.saturating_add(1);
                if errors == MAX_CONSECUTIVE_ERRORS {
                    error!("{} offline; retrying reads after backoff", id);
                    BAROMETER_STATUS[id.index()].store(SensorStatus::Disabled, Ordering::Relaxed);
                }
                if errors >= MAX_CONSECUTIVE_ERRORS {
                    buses::recover(bus).await;
                    Timer::after(I2C_RECOVERY_INTERVAL).await;
                    continue;
                }
            }
        }

        if Instant::now() > next_sample {
            warn!("{} can't keep up with sample interval", id);
        } else {
            Timer::at(next_sample).await;
        }
    }
}
