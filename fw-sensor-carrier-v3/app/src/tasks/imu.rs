use defmt::{info, warn};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::mode::Async;
use embassy_time::{Delay, Duration, Timer};
use lsm6dso32::spi::Lsm6Dso32SpiInterface;
use lsm6dso32::{
    AccelerometerFullScale, AccelerometerOdr, GyroscopeFullScale, GyroscopeOdr, Int1Config,
    Lsm6dso32,
};

use crate::resources::sensors::SpiDevice;

async fn run_inner<SPI, INT>(spi: SPI, mut int1: INT) -> !
where
    SPI: embedded_hal_async::spi::SpiDevice,
    INT: embedded_hal_async::digital::Wait,
{
    let mut iface = Lsm6Dso32SpiInterface { spi };

    // Init with retry
    let mut sensor = loop {
        let uninit = Lsm6dso32::<_, lsm6dso32::Uninitialised>::new(iface);
        match uninit.init(&mut Delay).await {
            Ok(s) => break s,
            Err(err) => {
                warn!("imu: init failed, retrying...");
                iface = err.sensor.destroy();
                Timer::after(Duration::from_millis(500)).await;
            }
        }
    };

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

    info!("imu: initialized, entering read loop");

    loop {
        let _ = int1.wait_for_rising_edge().await;

        match sensor.read_accelerometer_and_gyroscope().await {
            Ok((accel, gyro)) => {
                info!(
                    "imu: accel x={} y={} z={} | gyro x={} y={} z={}",
                    accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z
                );
            }
            Err(_) => {
                warn!("imu: read error");
            }
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
pub async fn task(spi: SpiDevice, int1: ExtiInput<'static, Async>) -> ! {
    run_inner(spi, int1).await
}
