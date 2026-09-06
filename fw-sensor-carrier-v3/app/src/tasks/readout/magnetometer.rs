use core::future::pending;
use core::sync::atomic::Ordering;

use defmt::{Debug2Format, debug, error, info, trace, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Instant, Timer};
use lsm303agr::{AccelMode, AccelOutputDataRate, Lsm303agr, MagMode, MagOutputDataRate};

use super::{MAX_CONSECUTIVE_ERRORS, MAX_INIT_ATTEMPTS, backoff};
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

type BusDevice = I2cDevice<'static, CriticalSectionRawMutex, SharedI2c>;
/// A fully-initialized magnetometer, ready to read.
pub type Sensor =
    Lsm303agr<lsm303agr::interface::I2cInterface<BusDevice>, lsm303agr::mode::MagContinuous>;

async fn configure(i2c: BusDevice, id: MagnetometerId) -> Result<Sensor, ()> {
    let mut sensor = Lsm303agr::new_with_i2c(i2c);

    if let Err(e) = sensor.init().await {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(());
    }

    let mut sensor = match sensor.into_mag_continuous().await {
        Ok(s) => s,
        Err(e) => {
            error!("{} init failed: {:?}", id, Debug2Format(&e.error));
            return Err(());
        }
    };

    if let Err(e) = sensor
        .set_mag_mode_and_odr(&mut Delay, MagMode::HighResolution, MAG_ODR)
        .await
    {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(());
    }

    if let Err(e) = sensor.enable_mag_offset_cancellation().await {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(());
    }

    if let Err(e) = sensor.mag_enable_low_pass_filter().await {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(());
    }

    if let Err(e) = sensor
        .set_accel_mode_and_odr(&mut Delay, AccelMode::Normal, AccelOutputDataRate::Hz50)
        .await
    {
        error!("{} init failed: {:?}", id, Debug2Format(&e));
        return Err(());
    }

    Ok(sensor)
}

/// Bring the magnetometer up, retrying a bounded number of times. Returns the live
/// sensor, or `None` (and marks it `Disabled`) if it never answered. Call this
/// sequentially at startup, before any read task runs, so a stuck bus can't starve
/// a healthy one mid-transaction.
pub async fn init(bus: SharedI2cBus, id: MagnetometerId) -> Option<Sensor> {
    for attempt in 1..=MAX_INIT_ATTEMPTS {
        debug!("{} initializing (attempt {})", id, attempt);
        if let Ok(sensor) = configure(I2cDevice::new(bus), id).await {
            info!("{} initialized", id);
            return Some(sensor);
        }
        if attempt < MAX_INIT_ATTEMPTS {
            Timer::after(backoff(attempt)).await;
        }
    }
    warn!("{} not detected; not polling", id);
    MAGNETOMETER_STATUS[id.index()].store(SensorStatus::Disabled, Ordering::Relaxed);
    None
}

#[embassy_executor::task(pool_size = 2)]
pub async fn read_task(mut sensor: Sensor, id: MagnetometerId) -> ! {
    MAGNETOMETER_STATUS[id.index()].store(SensorStatus::Active, Ordering::Relaxed);
    let mut errors: u8 = 0;

    loop {
        let next_sample = Instant::now() + SAMPLE_INTERVAL;

        match sensor.magnetic_field().await {
            Ok(field) => {
                errors = 0;
                let (x, y, z) = field.xyz_unscaled();
                let raw = RawMagSample {
                    src: id,
                    ts: Instant::now(),
                    x,
                    y,
                    z,
                };
                signals::submit_raw_mag_sample(raw);
                signals::submit_mag_sample(calibration::mag::apply_calibration(raw));
                trace!("{} x={} y={} z={} LSB", id, raw.x, raw.y, raw.z);
            }
            Err(e) => {
                warn!("{} read error: {:?}", id, Debug2Format(&e));
                errors = errors.saturating_add(1);
                if errors >= MAX_CONSECUTIVE_ERRORS {
                    error!("{} offline; no longer polling", id);
                    MAGNETOMETER_STATUS[id.index()]
                        .store(SensorStatus::Disabled, Ordering::Relaxed);
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
