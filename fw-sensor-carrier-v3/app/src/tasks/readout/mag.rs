//! LSM303AGR magnetometer readout on a shared I2C bus.

use defmt::{Debug2Format, debug, error, info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr, MagMode, MagOutputDataRate};

use super::{MAX_CONSECUTIVE_ERRORS, State, backoff, init_at_startup, wait_for_sample};
use crate::calibration;
use crate::resources::buses::{self, SharedI2c, SharedI2cBus};
use crate::sensors::{MAG_STATUS, MagId};
use crate::signals;
use crate::types::{RawMagSample, SdLogRecord};

const MAG_ODR: MagOutputDataRate = MagOutputDataRate::Hz10;
const SAMPLE_HZ: u32 = match MAG_ODR {
    MagOutputDataRate::Hz10 => 10,
    MagOutputDataRate::Hz20 => 20,
    MagOutputDataRate::Hz50 => 50,
    MagOutputDataRate::Hz100 => 100,
};
const SAMPLE_INTERVAL: Duration = Duration::from_millis(1000 / SAMPLE_HZ as u64);

type BusDevice = I2cDevice<'static, CriticalSectionRawMutex, SharedI2c>;
/// A fully-initialized magnetometer, ready to read.
pub type Sensor =
    Lsm303agr<lsm303agr::interface::I2cInterface<BusDevice>, lsm303agr::mode::MagContinuous>;

async fn configure(bus: SharedI2cBus, id: MagId) -> Result<Sensor, ()> {
    let mut sensor = Lsm303agr::new_with_i2c(I2cDevice::new(bus));
    sensor
        .init()
        .await
        .map_err(|e| error!("{}: init failed: {:?}", id, Debug2Format(&e)))?;
    let mut sensor = sensor
        .into_mag_continuous()
        .await
        .map_err(|e| error!("{}: init failed: {:?}", id, Debug2Format(&e.error)))?;
    sensor
        .set_mag_mode_and_odr(&mut Delay, MagMode::HighResolution, MAG_ODR)
        .await
        .map_err(|e| error!("{}: init failed: {:?}", id, Debug2Format(&e)))?;
    sensor
        .enable_mag_offset_cancellation()
        .await
        .map_err(|e| error!("{}: init failed: {:?}", id, Debug2Format(&e)))?;
    sensor
        .mag_enable_low_pass_filter()
        .await
        .map_err(|e| error!("{}: init failed: {:?}", id, Debug2Format(&e)))?;
    sensor
        .set_accel_mode_and_odr(&mut Delay, AccelMode::Normal, AccelOutputDataRate::Hz50)
        .await
        .map_err(|e| error!("{}: init failed: {:?}", id, Debug2Format(&e)))?;
    Ok(sensor)
}

/// Brings the magnetometer up at startup; see [`super`] for why this is separate.
pub async fn init(bus: SharedI2cBus, id: MagId) -> Option<Sensor> {
    init_at_startup(id, async || configure(bus, id).await).await
}

struct Inactive {
    /// Initialized by [`init`] at startup, if it succeeded.
    sensor: Option<Sensor>,
    bus: SharedI2cBus,
    id: MagId,
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
    id: MagId,
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
        let field = self
            .sensor
            .magnetic_field()
            .await
            .map_err(|e| warn!("{}: read error: {:?}", self.id, Debug2Format(&e)))?;
        let (x, y, z) = field.xyz_unscaled();
        let read_ts = Instant::now();
        let raw = RawMagSample {
            src: self.id,
            ts: read_ts,
            read_ts,
            x,
            y,
            z,
        };
        let cal = calibration::mag::apply_calibration(raw);
        signals::submit_raw_mag_sample(raw);
        signals::submit_mag_sample(cal);
        signals::submit_sd_log(SdLogRecord::Mag { raw, cal });
        Ok(())
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(sensor: Option<Sensor>, bus: SharedI2cBus, id: MagId) -> ! {
    let inactive = Inactive {
        sensor,
        bus,
        id,
        attempt: super::MAX_INIT_ATTEMPTS,
    };
    super::run(&MAG_STATUS[id.index()], inactive).await
}
