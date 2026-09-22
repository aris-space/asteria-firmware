use crate::drivers::WATCH;
use datatypes::status::SensorStatus;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Sender, Watch};

pub static DIGITAL_PRESSURE_WATCH: Watch<ThreadModeRawMutex, DigitalPressureMeasurementRaw, WATCH> =
    Watch::new();
pub static DPR_PRESSURE_WATCH: Watch<ThreadModeRawMutex, f32, WATCH> = Watch::new();
pub static KELLER_BUS_ERROR_WATCH: Watch<ThreadModeRawMutex, SensorStatus, WATCH> = Watch::new();

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DigitalPressureMeasurementRaw {
    pub prz_mnl_p: f32,
    pub fss_tnk_p1: f32,
    pub fss_tnk_p2: f32,
}

pub struct DigitalPressureDriver<'a> {
    digital_watch_handle: Sender<'a, ThreadModeRawMutex, DigitalPressureMeasurementRaw, WATCH>,
    dpr_watch_handle: Sender<'a, ThreadModeRawMutex, f32, WATCH>,
}

impl<'a> DigitalPressureDriver<'a> {
    pub fn new() -> Self {
        let digital_watch_handle = DIGITAL_PRESSURE_WATCH.sender();
        let dpr_watch_handle = DPR_PRESSURE_WATCH.sender();

        DigitalPressureDriver {
            digital_watch_handle,
            dpr_watch_handle,
        }
    }

    pub fn update(&mut self, value: DigitalPressureMeasurementRaw) {
        // Send the value to the watch channel
        self.digital_watch_handle.send(value);

        // Also send the DPR pressure to its own watch channel
        let filtered_p = get_filtered_tank_p(value.fss_tnk_p1, value.fss_tnk_p2);
        self.dpr_watch_handle.send(filtered_p);
    }
}

fn get_filtered_tank_p(p1: f32, p2: f32) -> f32 {
    if p1.is_nan() && p2.is_nan() {
        f32::INFINITY
    } else if p1.is_nan() {
        p2
    } else if p2.is_nan() {
        p1
    } else {
        f32::max(p1, p2)
    }
}
