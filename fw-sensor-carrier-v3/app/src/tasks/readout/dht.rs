//! SHT4x humidity and temperature readout on a shared I2C bus.

use defmt::{Debug2Format, debug, error, info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use sht4x::{Precision, Sht4xAsync};

use super::{MAX_CONSECUTIVE_ERRORS, State, backoff, init_at_startup, wait_for_sample};
use crate::calibration;
use crate::resources::buses::{self, SharedI2c, SharedI2cBus};
use crate::sensors::{DHT_STATUS, DhtId};
use crate::signals;
use crate::types::{RawDhtSample, SdLogRecord};

const SAMPLE_HZ: u32 = 1;
const SAMPLE_INTERVAL: Duration = Duration::from_millis(1000 / SAMPLE_HZ as u64);

type BusDevice = I2cDevice<'static, CriticalSectionRawMutex, SharedI2c>;
/// A fully-initialized humidity/temperature sensor, ready to read.
pub type Sensor = Sht4xAsync<BusDevice, Delay>;

async fn configure(bus: SharedI2cBus, id: DhtId) -> Result<Sensor, ()> {
    let mut sensor = Sht4xAsync::new(I2cDevice::new(bus));
    sensor
        .soft_reset(&mut Delay)
        .await
        .map_err(|e| error!("{}: init failed: {:?}", id, Debug2Format(&e)))?;
    sensor
        .measure(Precision::Low, &mut Delay)
        .await
        .map_err(|e| error!("{}: init failed: {:?}", id, Debug2Format(&e)))?;
    Ok(sensor)
}

/// Brings the DHT up at startup; see [`super`] for why this is separate.
pub async fn init(bus: SharedI2cBus, id: DhtId) -> Option<Sensor> {
    init_at_startup(id, async || configure(bus, id).await).await
}

struct Inactive {
    /// Initialized by [`init`] at startup, if it succeeded.
    sensor: Option<Sensor>,
    bus: SharedI2cBus,
    id: DhtId,
    attempt: u8,
}

impl State for Inactive {
    type Next = Active;

    async fn run(mut self) -> Active {
        loop {
            let sensor = match self.sensor.take() {
                Some(sensor) => Ok(sensor),
                None => {
                    debug!("{}: initializing", self.id);
                    configure(self.bus, self.id).await
                }
            };
            if let Ok(sensor) = sensor {
                info!("{}: active", self.id);
                return Active {
                    sensor,
                    bus: self.bus,
                    id: self.id,
                };
            }
            buses::recover(self.bus).await;
            self.attempt = self.attempt.saturating_add(1);
            Timer::after(backoff(self.attempt)).await;
        }
    }
}

struct Active {
    sensor: Sensor,
    bus: SharedI2cBus,
    id: DhtId,
}

impl State for Active {
    type Next = Inactive;

    async fn run(mut self) -> Inactive {
        let mut errors: u8 = 0;
        while errors < MAX_CONSECUTIVE_ERRORS {
            let next_sample = Instant::now() + SAMPLE_INTERVAL;
            match self.read().await {
                Ok(()) => errors = 0,
                Err(()) => errors += 1,
            }
            wait_for_sample(next_sample, self.id).await;
        }
        error!("{}: offline (too many consecutive errors)", self.id);
        Inactive {
            sensor: None,
            bus: self.bus,
            id: self.id,
            attempt: 0,
        }
    }
}

impl Active {
    async fn read(&mut self) -> Result<(), ()> {
        let m = self
            .sensor
            .measure(Precision::Low, &mut Delay)
            .await
            .map_err(|e| warn!("{}: read error: {:?}", self.id, Debug2Format(&e)))?;
        let read_ts = Instant::now();
        let raw = RawDhtSample {
            src: self.id,
            ts: read_ts,
            read_ts,
            temperature_c: m.temperature_celsius().to_num(),
            humidity_rh: m.humidity_percent().to_num(),
        };
        let cal = calibration::dht::apply_calibration(raw);
        signals::submit_dht_sample(cal);
        signals::submit_sd_log(SdLogRecord::Dht { raw, cal });
        Ok(())
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(sensor: Option<Sensor>, bus: SharedI2cBus, id: DhtId) -> ! {
    let inactive = Inactive {
        sensor,
        bus,
        id,
        attempt: super::MAX_INIT_ATTEMPTS,
    };
    super::run(&DHT_STATUS[id.index()], inactive).await
}
