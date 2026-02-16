use crate::drivers::WATCH;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Sender, Watch};
use hermes_can::messages::board_status::SensorStatus;

pub static THERMOCOUPLE_WATCH: Watch<ThreadModeRawMutex, ThermoMeasurementRaw, WATCH> =
    Watch::new();

pub static THERMOCOUPLE_ERROR_WATCH: Watch<ThreadModeRawMutex, SensorStatus, WATCH> = Watch::new();

#[derive(Clone, Copy, Debug)]
pub struct ThermoMeasurementRaw {
    pub fss_tnk_t: f32,
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
    }
}
