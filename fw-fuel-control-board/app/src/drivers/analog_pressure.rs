use crate::drivers::WATCH;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Sender, Watch};
use filters::GaussianMovingAverage;

const FILTER_WINDOW: usize = 10;
const FILTER_MEAN: f32 = 3.0;
const FILTER_SIGMA: f32 = 9.0;

pub static ANALOG_PRESSURE_WATCH: Watch<ThreadModeRawMutex, AnalogPressureMeasurementFiltered, WATCH,> = Watch::new();
pub static DPR_PRESSURE_WATCH: Watch<ThreadModeRawMutex, f32, WATCH> = Watch::new();


#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnalogPressureMeasurementRaw {
    pub prz_mnl_p: f32,
    pub fss_tnk_p1: f32,
    pub fss_tnk_p2: f32,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AnalogPressureMeasurementFiltered {
    pub prz_mnl_p: f32,
    pub fss_tnk_p1: f32,
    pub fss_tnk_p2: f32,
}

impl From<AnalogPressureMeasurementRaw> for AnalogPressureMeasurementFiltered {
    fn from(raw: AnalogPressureMeasurementRaw) -> Self {
        AnalogPressureMeasurementFiltered {
            prz_mnl_p: raw.prz_mnl_p,
            fss_tnk_p1: raw.fss_tnk_p1,
            fss_tnk_p2: raw.fss_tnk_p2,
        }
    }
}

pub struct AnalogPressureDriver<'a> {
    analog_watch_handle: Sender<'a, ThreadModeRawMutex, AnalogPressureMeasurementFiltered, WATCH>,
    dpr_watch_handle: Sender<'a, ThreadModeRawMutex, f32, WATCH>,
    prz_mnl_p_avg: GaussianMovingAverage<FILTER_WINDOW>,
    fss_tnk_p1_avg: GaussianMovingAverage<FILTER_WINDOW>,
    fss_tnk_p2_avg: GaussianMovingAverage<FILTER_WINDOW>,
}

impl<'a> AnalogPressureDriver<'a> {
    pub fn new() -> Self {
        let analog_watch_handle = ANALOG_PRESSURE_WATCH.sender();
        let dpr_watch_handle = DPR_PRESSURE_WATCH.sender();

        AnalogPressureDriver {
            analog_watch_handle,
            dpr_watch_handle,
            prz_mnl_p_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            fss_tnk_p1_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            fss_tnk_p2_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
        }
    }

    pub fn update(&mut self, mut value: AnalogPressureMeasurementRaw) {
        // Apply filtering to the raw sensor data
        value.prz_mnl_p = self.prz_mnl_p_avg.update(value.prz_mnl_p);
        value.fss_tnk_p1 = self.fss_tnk_p1_avg.update(value.fss_tnk_p1);
        value.fss_tnk_p2 = self.fss_tnk_p2_avg.update(value.fss_tnk_p2);

        // Send the value to the watch channel
        self.analog_watch_handle.send(value.into());

        // Update DPR watch channel
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

