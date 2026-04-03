use defmt::{info, trace, warn};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_time::{Delay, Timer};
use lsm6dso32::spi::Lsm6Dso32SpiInterface;
use lsm6dso32::{
    AccelerometerFullScale, AccelerometerOdr, GyroscopeFullScale, GyroscopeOdr, Int1Config,
    Lsm6dso32,
};

use super::{MAX_CONSECUTIVE_ERRORS, backoff};
use crate::resources::sensors::SpiDevice;

async fn configure(
    sensor: &mut Lsm6dso32<Lsm6Dso32SpiInterface<impl embedded_hal_async::spi::SpiDevice>, lsm6dso32::Initialised>,
) {
    if sensor
        .set_accelerometer_odr_and_full_scale(
            Some(AccelerometerOdr::Hz104),
            Some(AccelerometerFullScale::G8),
        )
        .await
        .is_err()
    {
        warn!("imu: failed to configure accelerometer");
    }
    if sensor
        .set_gyroscope_odr_and_full_scale(
            Some(GyroscopeOdr::Hz104),
            Some(GyroscopeFullScale::Dps2000),
        )
        .await
        .is_err()
    {
        warn!("imu: failed to configure gyroscope");
    }
    if sensor
        .configure_interrupts(
            Some(Int1Config {
                drdy_xl: true,
                ..Default::default()
            }),
            None,
        )
        .await
        .is_err()
    {
        warn!("imu: failed to configure interrupts");
    }
}

async fn run_inner<SPI, INT>(spi: SPI, mut int1: INT) -> !
where
    SPI: embedded_hal_async::spi::SpiDevice,
    INT: embedded_hal_async::digital::Wait,
{
    let mut iface = Lsm6Dso32SpiInterface { spi };
    let mut attempt: u8 = 0;

    loop {
        let mut sensor = loop {
            let uninit = Lsm6dso32::<_, lsm6dso32::Uninitialised>::new(iface);
            match uninit.init(&mut Delay).await {
                Ok(s) => {
                    attempt = 0;
                    break s;
                }
                Err(err) => {
                    attempt = attempt.saturating_add(1);
                    warn!("imu: init failed (attempt {}), retrying...", attempt);
                    iface = err.sensor.destroy();
                    Timer::after(backoff(attempt)).await;
                }
            }
        };

        configure(&mut sensor).await;
        info!("imu: active");

        let mut errors: u8 = 0;
        loop {
            let _ = int1.wait_for_rising_edge().await;

            match sensor.read_accelerometer_and_gyroscope().await {
                Ok((accel, gyro)) => {
                    errors = 0;
                    trace!(
                        "imu: accel x={} y={} z={} | gyro x={} y={} z={}",
                        accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z
                    );
                }
                Err(_) => {
                    errors = errors.saturating_add(1);
                    warn!("imu: read error ({}/{})", errors, MAX_CONSECUTIVE_ERRORS);
                    if errors >= MAX_CONSECUTIVE_ERRORS {
                        break;
                    }
                }
            }
        }

        warn!("imu: inactive (too many errors)");
        iface = sensor.destroy();
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(spi: SpiDevice, int1: ExtiInput<'static, Async>) -> ! {
    run_inner(spi, int1).await
}
