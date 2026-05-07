use crate::drivers::WATCH;
use crate::globals::STATE;
use datatypes::status::SensorStatus;
use datatypes::units::Celsius;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Sender, Watch};

pub static THERMOCOUPLE_WATCH: Watch<ThreadModeRawMutex, ThermoMeasurementRaw, WATCH> =
    Watch::new();

pub static THERMOCOUPLE_ERROR_WATCH: Watch<ThreadModeRawMutex, SensorStatus, WATCH> = Watch::new();

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ThermoMeasurementRaw {
    pub fss_inj_t: f32,
    pub oss_tnk_t: f32,
}

pub struct TCDriver<'a> {
    watch_handle: Sender<'a, ThreadModeRawMutex, ThermoMeasurementRaw, WATCH>,
}

impl<'a> TCDriver<'a> {
    pub fn new() -> Self {
        let watch_handle = THERMOCOUPLE_WATCH.sender();

        TCDriver { watch_handle }
    }

    pub fn update(&mut self, value: ThermoMeasurementRaw) {
        // Send the value to the watch channel
        self.watch_handle.send(value);
        STATE
            .engine_bay_temperature
            .sender()
            .send(dp_engine_control_board::EngineBayTemperature {
                eng_inj_t: Celsius(value.fss_inj_t),
                oss_tnk_t: Celsius(value.oss_tnk_t),
            });
    }
}
