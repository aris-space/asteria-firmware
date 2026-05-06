use crate::globals::STATE;
use datatypes::status::SensorStatus;
use datatypes::units::BarG;
use filters::GaussianMovingAverage;

const FILTER_WINDOW: usize = 10;
const FILTER_MEAN: f32 = 3.0;
const FILTER_SIGMA: f32 = 9.0;

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OxidizerPressureMeasurementRaw {
    pub oxidizer_tank_pressure_1: f32,
    pub oxidizer_tank_pressure_2: f32,
    pub oxidizer_tank_differential_pressure: f32,
}

pub struct OxidizerPressureDriver {
    oxidizer_tank_pressure_1_avg: GaussianMovingAverage<FILTER_WINDOW>,
    oxidizer_tank_pressure_2_avg: GaussianMovingAverage<FILTER_WINDOW>,
    oxidizer_tank_differential_pressure_avg: GaussianMovingAverage<FILTER_WINDOW>,
}

impl OxidizerPressureDriver {
    pub fn new() -> Self {
        Self {
            oxidizer_tank_pressure_1_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            oxidizer_tank_pressure_2_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            oxidizer_tank_differential_pressure_avg: GaussianMovingAverage::new(
                FILTER_SIGMA,
                FILTER_MEAN,
            ),
        }
    }

    pub fn update(&mut self, mut value: OxidizerPressureMeasurementRaw) {
        let has_error = !value.oxidizer_tank_pressure_1.is_finite()
            || !value.oxidizer_tank_pressure_2.is_finite()
            || !value.oxidizer_tank_differential_pressure.is_finite();

        value.oxidizer_tank_pressure_1 = update_if_finite(
            &mut self.oxidizer_tank_pressure_1_avg,
            value.oxidizer_tank_pressure_1,
        );
        value.oxidizer_tank_pressure_2 = update_if_finite(
            &mut self.oxidizer_tank_pressure_2_avg,
            value.oxidizer_tank_pressure_2,
        );
        value.oxidizer_tank_differential_pressure = update_if_finite(
            &mut self.oxidizer_tank_differential_pressure_avg,
            value.oxidizer_tank_differential_pressure,
        );

        STATE.oxidizer_tank_pressure.sender().send(
            dp_oxidizer_control_board::OxidizerTankPressure {
                oxidizer_tank_pressure_sensor_1: BarG(value.oxidizer_tank_pressure_1),
                oxidizer_tank_pressure_sensor_2: BarG(value.oxidizer_tank_pressure_2),
                oxidizer_tank_differential_pressure: BarG(
                    value.oxidizer_tank_differential_pressure,
                ),
            },
        );

        STATE.pressure_bus_status.sender().send(if has_error {
            SensorStatus::Offline
        } else {
            SensorStatus::Online
        });
    }
}

fn update_if_finite(avg: &mut GaussianMovingAverage<FILTER_WINDOW>, value: f32) -> f32 {
    if value.is_finite() {
        avg.update(value)
    } else {
        value
    }
}
