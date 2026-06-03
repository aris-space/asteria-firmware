use crate::drivers::ENGINE_P_WATCH;
use crate::globals::STATE;
use datatypes::units::BarG;
use filters::GaussianMovingAverage;

const FILTER_WINDOW: usize = 10;
const FILTER_MEAN: f32 = 3.0;
const FILTER_SIGMA: f32 = 9.0;

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EnginePressureMeasurementRaw {
    pub eng_cc_p: f32,
    pub fss_inj_p: f32,
    pub oss_inj_p: f32,
}

pub struct EnginePressureDriver {
    eng_cc_p_avg: GaussianMovingAverage<FILTER_WINDOW>,
    oss_inj_p_avg: GaussianMovingAverage<FILTER_WINDOW>,
    fss_inj_p_avg: GaussianMovingAverage<FILTER_WINDOW>,
}

impl EnginePressureDriver {
    pub fn new() -> Self {
        EnginePressureDriver {
            eng_cc_p_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            oss_inj_p_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            fss_inj_p_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
        }
    }

    pub fn update(
        &mut self,
        mut value: EnginePressureMeasurementRaw,
    ) -> EnginePressureMeasurementRaw {
        // Apply filtering to the raw sensor data
        value.eng_cc_p = self.eng_cc_p_avg.update(value.eng_cc_p);
        value.oss_inj_p = self.oss_inj_p_avg.update(value.oss_inj_p);
        value.fss_inj_p = self.fss_inj_p_avg.update(value.fss_inj_p);

        STATE.eng_cc_p.sender().send(BarG(value.eng_cc_p));
        STATE.fss_inj_p.sender().send(BarG(value.fss_inj_p));
        STATE.oss_inj_p.sender().send(BarG(value.oss_inj_p));

        // Update the engine chamber pressure watch
        ENGINE_P_WATCH.sender().send(value.eng_cc_p);
        STATE.engine_chamber_pressure.sender().send(value.eng_cc_p);

        value
    }
}
