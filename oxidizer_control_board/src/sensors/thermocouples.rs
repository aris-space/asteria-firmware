use embassy_time::{Duration, Ticker};
use hermes_can::messages::board_status::SensorStatus;
use max31889_thermistor::MAX31889;

use crate::drivers::temperature::{TCDriver, ThermoMeasurementRaw, THERMOCOUPLE_ERROR_WATCH};
use crate::sensors::ACQ_THERMOCOUPLE_FREQ_HZ;
use ads1120_thermocouples::thermocouple_conversions::{ThermocoupleConversion, ThermocoupleType};
use ads1120_thermocouples::ADSThermocouples;
use embedded_utils::error;

#[embassy_executor::task]
pub async fn thermocouple_task(
    mut ads: ADSThermocouples<'static>,
    mut max: MAX31889<'static>,
    tc_primary: &'static ThermocoupleType,
    _tc_secondary: &'static ThermocoupleType,
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

        // Convert Primary Thermocouple readings
        let cold_junction_primary_v = tc_primary.temperature_to_voltage(cold_junction_t);
        let tc_primary_temp =
            tc_primary.voltage_to_temperature(tc_primary_voltage, cold_junction_primary_v);

        let measurement = ThermoMeasurementRaw {
            fss_tnk_t: tc_primary_temp,
        };

        driver.update(measurement);

        if error_count > 0 {
            error!("[TC] Thermocouple errors detected: {}", error_count);
            status_sender.send(SensorStatus::Offline);
            error_count = 0; // Reset error count after sending
        } else {
            status_sender.send(SensorStatus::Online);
        }
        ticker.next().await;
    }
}
