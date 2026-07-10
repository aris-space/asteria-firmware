use core::sync::atomic::Ordering;

use defmt::{Debug2Format, debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr, MagMode, MagOutputDataRate};

use super::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::calibration;
use crate::resources::buses::{SharedI2c, SharedI2cBus};
use crate::sensors::{MAGNETOMETER_STATUS, MagnetometerId, SensorStatus};
use crate::signals;
use crate::types::RawMagSample;

const MAG_ODR: MagOutputDataRate = MagOutputDataRate::Hz10;
pub const SAMPLE_HZ: u32 = match MAG_ODR {
    MagOutputDataRate::Hz10 => 10,
    MagOutputDataRate::Hz20 => 20,
    MagOutputDataRate::Hz50 => 50,
    MagOutputDataRate::Hz100 => 100,
};
const SAMPLE_INTERVAL: Duration = Duration::from_millis(1000 / SAMPLE_HZ as u64);

async fn initialise<I2C: embedded_hal_async::i2c::I2c>(
    i2c: I2C,
    id: MagnetometerId,
) -> Result<Lsm303agr<lsm303agr::interface::I2cInterface<I2C>, lsm303agr::mode::MagContinuous>, I2C>
{
    let mut sensor = Lsm303agr::new_with_i2c(i2c);

    if let Err(e) = sensor.init().await {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(sensor.destroy());
    }

    let mut sensor = match sensor.into_mag_continuous().await {
        Ok(s) => s,
        Err(e) => {
            error!("{} init failed: {:?}", id, Debug2Format(&e.error));
            return Err(e.dev.destroy());
        }
    };

    if let Err(e) = sensor
        .set_mag_mode_and_odr(&mut Delay, MagMode::HighResolution, MAG_ODR)
        .await
    {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(sensor.destroy());
    }

    if let Err(e) = sensor.enable_mag_offset_cancellation().await {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(sensor.destroy());
    }

    if let Err(e) = sensor.mag_enable_low_pass_filter().await {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(sensor.destroy());
    }

    if let Err(e) = sensor
        .set_accel_mode_and_odr(&mut Delay, AccelMode::Normal, AccelOutputDataRate::Hz50)
        .await
    {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(sensor.destroy());
    }

    Ok(sensor)
}

struct Inactive<I2C> {
    i2c: I2C,
    id: MagnetometerId,
    attempt: u8,
}

impl<I2C: embedded_hal_async::i2c::I2c> Inactive<I2C> {
    async fn run(mut self) -> Active<I2C> {
        loop {
            debug!("{} initializing", self.id);
            match initialise(self.i2c, self.id).await {
                Ok(sensor) => {
                    info!("{} initialized", self.id);
                    return Active {
                        sensor,
                        id: self.id,
                    };
                }
                Err(i2c) => {
                    self.i2c = i2c;
                    self.attempt = self.attempt.saturating_add(1);
                    Timer::after(backoff(self.attempt)).await;
                }
            }
        }
    }
}

struct Active<I2C> {
    sensor: Lsm303agr<lsm303agr::interface::I2cInterface<I2C>, lsm303agr::mode::MagContinuous>,
    id: MagnetometerId,
}

impl<I2C: embedded_hal_async::i2c::I2c> Active<I2C> {
    async fn run(mut self) -> Inactive<I2C> {
        let mut errors: u8 = 0;

        loop {
            let next_sample = Instant::now() + SAMPLE_INTERVAL;

            match self.sensor.magnetic_field().await {
                Ok(field) => {
                    errors = 0;
                    let (x, y, z) = field.xyz_unscaled();
                    let raw = RawMagSample {
                        src: self.id,
                        ts: Instant::now(),
                        x,
                        y,
                        z,
                    };
                    signals::submit_raw_mag_sample(raw);
                    signals::submit_mag_sample(calibration::mag::apply_calibration(raw));
                    trace!("{} x={} y={} z={} LSB", self.id, raw.x, raw.y, raw.z);
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
            i2c: self.sensor.destroy(),
            id: self.id,
            attempt: 0,
        }
    }
}

async fn run_inner<I2C: embedded_hal_async::i2c::I2c>(i2c: I2C, id: MagnetometerId) -> ! {
    let mut inactive = Inactive {
        i2c,
        id,
        attempt: 0,
    };

    loop {
        let active = inactive.run().await;
        MAGNETOMETER_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
        inactive = active.run().await;
        MAGNETOMETER_STATUS[id.index()].store(SensorStatus::Inactive, Ordering::Relaxed);
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(bus: SharedI2cBus, id: MagnetometerId) -> ! {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    run_inner(i2c, id).await
}
