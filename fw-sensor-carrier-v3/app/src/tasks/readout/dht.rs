use core::future::pending;
use core::sync::atomic::Ordering;

use defmt::{Debug2Format, debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use sht4x::{Precision, Sht4xAsync};

use super::{MAX_CONSECUTIVE_ERRORS, MAX_INIT_ATTEMPTS, backoff};
use crate::calibration;
use crate::resources::buses::{SharedI2c, SharedI2cBus};
use crate::sensors::{DHT_STATUS, DhtId, SensorStatus};
use crate::signals;
use crate::types::RawDhtSample;

pub const SAMPLE_HZ: u32 = 1;
const SAMPLE_INTERVAL: Duration = Duration::from_millis(1000 / SAMPLE_HZ as u64);

type BusDevice = I2cDevice<'static, CriticalSectionRawMutex, SharedI2c>;
/// A fully-initialized humidity/temperature sensor, ready to read.
pub type Sensor = Sht4xAsync<BusDevice, Delay>;

/// Bring the DHT up, retrying a bounded number of times. Returns the live sensor,
/// or `None` (and marks it `Disabled`) if it never answered. Call this sequentially
/// at startup, before any read task runs, so a stuck bus can't starve a healthy one
/// mid-transaction.
pub async fn init(bus: SharedI2cBus, id: DhtId) -> Option<Sensor> {
    for attempt in 1..=MAX_INIT_ATTEMPTS {
        debug!("{} initializing (attempt {})", id, attempt);
        let mut sensor = Sht4xAsync::new(I2cDevice::new(bus));
        let result = match sensor.soft_reset(&mut Delay).await {
            Ok(()) => sensor.measure(Precision::Low, &mut Delay).await.map(|_| ()),
            Err(e) => Err(e),
        };
        match result {
            Ok(()) => {
                info!("{} initialized", id);
                return Some(sensor);
            }
            Err(e) => {
                error!("{} init failed: {:?}", id, Debug2Format(&e));
                if attempt < MAX_INIT_ATTEMPTS {
                    Timer::after(backoff(attempt)).await;
                }
            }
        }
    }
    warn!("{} not detected; not polling", id);
    DHT_STATUS[id.index()].store(SensorStatus::Disabled, Ordering::Relaxed);
    None
}

#[embassy_executor::task(pool_size = 2)]
pub async fn read_task(mut sensor: Sensor, id: DhtId) -> ! {
    DHT_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
    let mut errors: u8 = 0;

    loop {
        let next_sample = Instant::now() + SAMPLE_INTERVAL;

        match sensor.measure(Precision::Low, &mut Delay).await {
            Ok(m) => {
                errors = 0;
                let temperature_c: f32 = m.temperature_celsius().to_num();
                let humidity_rh: f32 = m.humidity_percent().to_num();
                let raw = RawDhtSample {
                    src: id,
                    ts: Instant::now(),
                    temperature_c,
                    humidity_rh,
                };
                signals::submit_dht_sample(calibration::dht::apply_calibration(raw));
                trace!("{} t={} c rh={} %", id, temperature_c, humidity_rh);
            }
            Err(e) => {
                warn!("{} read error: {:?}", id, Debug2Format(&e));
                errors = errors.saturating_add(1);
                if errors >= MAX_CONSECUTIVE_ERRORS {
                    error!("{} offline; no longer polling", id);
                    DHT_STATUS[id.index()].store(SensorStatus::Disabled, Ordering::Relaxed);
                    loop {
                        pending::<()>().await;
                    }
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
