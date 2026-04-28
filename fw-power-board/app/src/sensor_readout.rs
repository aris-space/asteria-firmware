use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker};
use embedded_utils::fmt::{error, info};
use hermes_can::messages::sensor_data::{RailStatus, RailStatus5V, RailStatus24V};
use ina232::I2cInterface;

pub static RAIL_5V_LAST: Watch<ThreadModeRawMutex, RailStatus5V, 2> = Watch::new();
pub static RAIL_24V_LAST: Watch<ThreadModeRawMutex, RailStatus24V, 2> = Watch::new();

/// 5 V rail readout task
#[embassy_executor::task]
pub async fn sensor_readout_5v_task(
    mut rail: ina232::Ina232<I2cInterface<I2c<'static, Async, i2c::mode::Master>>>,
) -> ! {
    let sender_5v = RAIL_5V_LAST.sender();
    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        // todo(@louis): add timeouts for reading.
        let voltage_res = rail.read_voltage().await;
        let current_res = rail.read_current().await;
        let power_res = rail.read_power().await;

        let all_err = [&voltage_res, &current_res, &power_res]
            .iter()
            .all(|res| res.is_err());

        let voltage = voltage_res.unwrap_or_else(|err| {
            error!("[5V] Failed to read voltage: {:?}", err);
            0.0
        });
        let current = current_res.unwrap_or_else(|err| {
            error!("[5V] Failed to read current: {:?}", err);
            0.0
        });
        let power = power_res.unwrap_or_else(|err| {
            error!("[5V] Failed to read power: {:?}", err);
            0.0
        });

        if !all_err {
            // there is at least some useful and valid data
            let status = RailStatus5V {
                data: RailStatus {
                    voltage,
                    current,
                    power,
                },
            };
            sender_5v.send(status.clone());
            info!("{:?}", status);
        }
        ticker.next().await; // wait for the next tick
    }
}

/// 24 V rail readout task
#[embassy_executor::task]
pub async fn sensor_readout_24v_task(
    mut rail: ina232::Ina232<I2cInterface<I2c<'static, Async, i2c::mode::Master>>>,
) -> ! {
    let sender_24v = RAIL_24V_LAST.sender();
    let mut ticker = Ticker::every(Duration::from_millis(100));
    loop {
        // todo(@louis): add timeouts for reading.
        let voltage_res = rail.read_voltage().await;
        let current_res = rail.read_current().await;
        let power_res = rail.read_power().await;

        let all_err = [&voltage_res, &current_res, &power_res]
            .iter()
            .all(|res| res.is_err());

        let voltage = voltage_res.unwrap_or_else(|err| {
            error!("[24V] Failed to read voltage: {:?}", err);
            0.0
        });
        let current = current_res.unwrap_or_else(|err| {
            error!("[24V] Failed to read current: {:?}", err);
            0.0
        });
        let power = power_res.unwrap_or_else(|err| {
            error!("[24V] Failed to read power: {:?}", err);
            0.0
        });

        if !all_err {
            // there is at least some useful and valid data
            let status = RailStatus24V {
                data: RailStatus {
                    voltage,
                    current,
                    power,
                },
            };
            sender_24v.send(status.clone());
            info!("{:?}", status);
        }
        ticker.next().await; // wait for the next tick
    }
}
