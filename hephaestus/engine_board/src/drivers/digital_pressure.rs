use crate::drivers::{ENGINE_P_WATCH, WATCH};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Sender, Watch};
use hermes_can::messages::board_status::SensorStatus;

pub static DIGITAL_PRESSURE_WATCH: Watch<ThreadModeRawMutex, DigitalPressureMeasurementRaw, WATCH> =
    Watch::new();

pub static DIGITAL_TEMPERATURE_WATCH: Watch<
    ThreadModeRawMutex,
    DigitalTemperatureMeasurementRaw,
    WATCH,
> = Watch::new();

pub static KELLER_BUS_ERROR_WATCH: Watch<ThreadModeRawMutex, SensorStatus, WATCH> = Watch::new();

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DigitalPressureMeasurementRaw {
    pub eng_cc_p: f32,
    pub fue_inj_p: f32,
    pub oxd_inj_p: f32,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DigitalTemperatureMeasurementRaw {
    pub eng_cc_t: f32,
    pub fue_inj_t: f32,
    pub oxd_inj_t: f32,
}

pub struct KellerDriver<'a> {
    digital_pressure_handle: Sender<'a, ThreadModeRawMutex, DigitalPressureMeasurementRaw, WATCH>,
    digital_temperature_handle:
        Sender<'a, ThreadModeRawMutex, DigitalTemperatureMeasurementRaw, WATCH>,
    eng_p_handle: Sender<'a, ThreadModeRawMutex, f32, WATCH>,
}

impl<'a> KellerDriver<'a> {
    pub fn new() -> Self {
        let digital_pressure_handle = DIGITAL_PRESSURE_WATCH.sender();
        let digital_temperature_handle = DIGITAL_TEMPERATURE_WATCH.sender();
        let eng_p_handle = ENGINE_P_WATCH.sender();

        KellerDriver {
            digital_pressure_handle,
            digital_temperature_handle,
            eng_p_handle,
        }
    }

    pub fn update(
        &mut self,
        pressure: DigitalPressureMeasurementRaw,
        temperature: DigitalTemperatureMeasurementRaw,
    ) {
        // Send the value to the watch channel
        self.digital_pressure_handle.send(pressure);
        self.digital_temperature_handle.send(temperature);

        self.eng_p_handle.send(pressure.eng_cc_p);
    }
}
