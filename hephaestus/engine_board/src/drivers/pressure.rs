#![allow(dead_code)]
use crate::drivers::{ENGINE_P_WATCH, WATCH};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Sender, Watch};
use filters::GaussianMovingAverage;

const FILTER_WINDOW: usize = 10;
const FILTER_MEAN: f32 = 3.0;
const FILTER_SIGMA: f32 = 9.0;

pub static ANALOG_PRESSURE_WATCH: Watch<
    ThreadModeRawMutex,
    AnalogPressureMeasurementFiltered,
    WATCH,
> = Watch::new();

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnalogPressureMeasurementRaw {
    pub eng_cc_p: f32,
    pub fss_inj_p: f32,
    pub oss_inj_p: f32,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnalogPressureMeasurementFiltered {
    pub eng_cc_p: f32,
    pub fss_inj_p: f32,
    pub oss_inj_p: f32,
}

impl From<AnalogPressureMeasurementRaw> for AnalogPressureMeasurementFiltered {
    fn from(raw: AnalogPressureMeasurementRaw) -> Self {
        AnalogPressureMeasurementFiltered {
            eng_cc_p: raw.eng_cc_p,
            fss_inj_p: raw.fss_inj_p,
            oss_inj_p: raw.oss_inj_p,
        }
    }
}

pub struct AnalogPressureDriver<'a> {
    watch_handle: Sender<'a, ThreadModeRawMutex, AnalogPressureMeasurementFiltered, WATCH>,
    engine_p_handle: Sender<'a, ThreadModeRawMutex, f32, WATCH>,
    eng_cc_p_avg: GaussianMovingAverage<FILTER_WINDOW>,
    oss_inj_p_avg: GaussianMovingAverage<FILTER_WINDOW>,
    fss_inj_p_avg: GaussianMovingAverage<FILTER_WINDOW>,
}

impl<'a> AnalogPressureDriver<'a> {
    pub fn new() -> Self {
        let watch_handle = ANALOG_PRESSURE_WATCH.sender();
        let engine_p_handle = ENGINE_P_WATCH.sender();

        AnalogPressureDriver {
            watch_handle,
            engine_p_handle,
            eng_cc_p_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            oss_inj_p_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            fss_inj_p_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
        }
    }

    pub fn update(&mut self, mut value: AnalogPressureMeasurementRaw) {
        // Apply filtering to the raw sensor data
        value.eng_cc_p = self.eng_cc_p_avg.update(value.eng_cc_p);
        value.oss_inj_p = self.oss_inj_p_avg.update(value.oss_inj_p);
        value.fss_inj_p = self.fss_inj_p_avg.update(value.fss_inj_p);

        // Send the value to the watch channel
        self.watch_handle.send(value.into());

        // Update the engine chamber pressure watch
        self.engine_p_handle.send(value.eng_cc_p);
    }
}
