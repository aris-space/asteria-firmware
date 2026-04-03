use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_time::{Delay, Duration, Timer};
use ms5607::{Ms5607, Oversampling};

use crate::resources::buses::{SharedI2c, SharedI2cBus};

async fn run_inner<I2C>(i2c: I2C) -> !
where
    I2C: embedded_hal_async::i2c::I2c,
{
    let mut uninit = Ms5607::new(i2c, false);

    let mut sensor = loop {
        match uninit.init(&mut Delay).await {
            Ok(s) => break s,
            Err(err) => {
                warn!("barometer: init failed, retrying...");
                let addr = err.sensor.address();
                let i2c = err.sensor.destroy();
                uninit = Ms5607::new_with_addr(i2c, addr);
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    };

    info!("barometer: initialized, entering read loop");

    loop {
        match sensor.measure(Oversampling::Osr2048, &mut Delay).await {
            Ok(m) => {
                info!(
                    "barometer: pressure={} mbar, temp={} C",
                    m.pressure_mbar, m.temperature_c
                );
            }
            Err(_) => {
                warn!("barometer: read error");
            }
        }

        Timer::after(Duration::from_millis(25)).await;
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(bus: SharedI2cBus) -> ! {
    let i2c = I2cDevice::<CriticalSectionRawMutex, SharedI2c>::new(bus);
    run_inner(i2c).await
}
