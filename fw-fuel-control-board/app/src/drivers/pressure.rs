use crate::globals::STATE;
use datatypes::units::BarG;
use filters::GaussianMovingAverage;

const FILTER_WINDOW: usize = 10;
const FILTER_MEAN: f32 = 3.0;
const FILTER_SIGMA: f32 = 9.0;

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FuelPressureMeasurementRaw {
    pub pressurization_pressure: f32,
    pub fuel_tank_pressure_1: f32,
    pub fuel_tank_pressure_2: f32,
}

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FuelTankPressureMeasurement {
    pub fuel_tank_pressure_1: BarG,
    pub fuel_tank_pressure_2: BarG,
    pub dpr_pressure: BarG,
}

pub struct FuelPressureDriver {
    pressurization_pressure_avg: GaussianMovingAverage<FILTER_WINDOW>,
    fuel_tank_pressure_1_avg: GaussianMovingAverage<FILTER_WINDOW>,
    fuel_tank_pressure_2_avg: GaussianMovingAverage<FILTER_WINDOW>,
}

impl FuelPressureDriver {
    pub fn new() -> Self {
        Self {
            pressurization_pressure_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            fuel_tank_pressure_1_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
            fuel_tank_pressure_2_avg: GaussianMovingAverage::new(FILTER_SIGMA, FILTER_MEAN),
        }
    }

    pub fn update(&mut self, mut value: FuelPressureMeasurementRaw) {
        value.pressurization_pressure = self
            .pressurization_pressure_avg
            .update(value.pressurization_pressure);
        value.fuel_tank_pressure_1 = self
            .fuel_tank_pressure_1_avg
            .update(value.fuel_tank_pressure_1);
        value.fuel_tank_pressure_2 = self
            .fuel_tank_pressure_2_avg
            .update(value.fuel_tank_pressure_2);

        STATE
            .pressurization_pressure
            .sender()
            .send(BarG(value.pressurization_pressure));
        STATE
            .fuel_tank_pressure
            .sender()
            .send(FuelTankPressureMeasurement {
                fuel_tank_pressure_1: BarG(value.fuel_tank_pressure_1),
                fuel_tank_pressure_2: BarG(value.fuel_tank_pressure_2),
                dpr_pressure: BarG(get_filtered_tank_p(
                    value.fuel_tank_pressure_1,
                    value.fuel_tank_pressure_2,
                )),
            });
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
