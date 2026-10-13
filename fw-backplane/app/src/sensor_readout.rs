use crate::can::OUTPUTS;
use datatypes::units::RailStatus;
use dp_backplane::ActivePowerSource;
use embassy_stm32::gpio::{Input, Level, Output};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_time::{Duration, Ticker};
use embedded_utils::fmt::{error, info};
use ina232::I2cInterface;

/// 5 V rail readout task
#[embassy_executor::task]
pub async fn sensor_readout_5v_task(
    mut rail: ina232::Ina232<I2cInterface<I2c<'static, Async, i2c::mode::Master>>>,
) -> ! {
    let sender_5v = OUTPUTS.rail_5v.sender();
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
            let status = RailStatus {
                voltage,
                current,
                power,
            };
            info!("{:?}", status);
            sender_5v.send(status);
        }
        ticker.next().await; // wait for the next tick
    }
}

/// 24 V rail readout task
#[embassy_executor::task]
pub async fn sensor_readout_24v_task(
    mut rail: ina232::Ina232<I2cInterface<I2c<'static, Async, i2c::mode::Master>>>,
) -> ! {
    let sender_24v = OUTPUTS.rail_24v.sender();
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
            let status = RailStatus {
                voltage,
                current,
                power,
            };
            info!("{:?}", status);
            sender_24v.send(status);
        }
        ticker.next().await; // wait for the next tick
    }
}

/// Readout digital inputs for battery & external power, set their LEDs,
/// and track which is active for CAN.
#[embassy_executor::task]
pub async fn active_power_source_task(
    bat_p: Input<'static>,
    ext_p: Input<'static>,
    mut led_bat_p: Output<'static>,
    mut led_ext_p: Output<'static>,
) {
    let active_power_sender = OUTPUTS.active_power_source.sender();
    let mut ticker = Ticker::every(Duration::from_millis(100));

    loop {
        let bat_level = bat_p.get_level();
        let ext_level = ext_p.get_level();
        led_bat_p.set_level(bat_level);
        led_ext_p.set_level(ext_level);

        let active_source = if ext_level == Level::High {
            ActivePowerSource::External
        } else if bat_level == Level::High {
            ActivePowerSource::Battery
        } else {
            ActivePowerSource::None
        };
        active_power_sender.send(active_source);

        ticker.next().await;
    }
}
