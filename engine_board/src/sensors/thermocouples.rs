use embassy_time::{Duration, Ticker};
use hermes_can::messages::board_status::SensorStatus;
use max31889_thermistor::MAX31889;

use crate::drivers::temperature::{TCDriver, ThermoMeasurementRaw, THERMOCOUPLE_ERROR_WATCH};
use crate::sensors::ACQ_THERMOCOUPLE_FREQ_HZ;
use ads1120_thermocouples::thermocouple_conversions::{ThermocoupleConversion, ThermocoupleType};
use ads1120_thermocouples::ADSThermocouples;
use embedded_utils::error;
use embedded_utils::fmt::warn;

#[embassy_executor::task]
pub async fn thermocouple_task(
    mut ads: ADSThermocouples<'static>,
    mut max: MAX31889<'static>,
    tc_primary: &'static ThermocoupleType,
    tc_secondary: &'static ThermocoupleType,
) {
    let mut driver = TCDriver::new();
    let status_sender = THERMOCOUPLE_ERROR_WATCH.sender();
    let mut error_count = 0;

    let mut ticker = Ticker::every(Duration::from_millis(
        (1000.0 / ACQ_THERMOCOUPLE_FREQ_HZ) as u64,
    ));

    loop {
        let tc_primary_voltage = ads.read_primary().await.unwrap_or_else(|e| {
            error!("Failed to read primary thermocouple voltage: {}", e);
            error_count += 1;
            f32::NAN
        });
        let cold_junction_t = max.combined_temperature_read().await.unwrap_or_else(|e| {
            error!("Failed to read cold junction temperature: {}", e);
            error_count += 1;
            f32::NAN
        });

        let tc_secondary_voltage = ads.read_secondary().await.unwrap_or_else(|e| {
            error!("Failed to read secondary thermocouple: {}", e);
            error_count += 1;
            f32::NAN
        });
        // Convert Primary Thermocouple readings
        let cold_junction_primary_v = tc_primary.temperature_to_voltage(cold_junction_t);
        let tc_primary_temp =
            tc_primary.voltage_to_temperature(tc_primary_voltage, cold_junction_primary_v);

        // Convert Secondary Thermocouple readings
        let cold_junction_secondary_v = tc_secondary.temperature_to_voltage(cold_junction_t);
        let tc_secondary_temp =
            tc_secondary.voltage_to_temperature(tc_secondary_voltage, cold_junction_secondary_v);

        let measurement = ThermoMeasurementRaw {
            oss_rnl_t: tc_primary_temp,
            oss_tnk_t: tc_secondary_temp,
        };

        driver.update(measurement);

        if error_count > 0 {
            warn!("[TC] Thermocouple errors detected: {}", error_count);
            status_sender.send(SensorStatus::Offline);
            error_count = 0; // Reset error count after sending
        } else {
            status_sender.send(SensorStatus::Online);
        }
        ticker.next().await;
    }
}
