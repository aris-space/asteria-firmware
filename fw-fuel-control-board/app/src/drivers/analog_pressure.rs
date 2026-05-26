use crate::globals::STATE;
use datatypes::status::SensorStatus;
use datatypes::units::BarG;
use embassy_sync::watch::Watch;
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
        let has_error = !value.pressurization_pressure.is_finite()
            || !value.fuel_tank_pressure_1.is_finite()
            || !value.fuel_tank_pressure_2.is_finite();

        value.pressurization_pressure = update_if_finite(
            &mut self.pressurization_pressure_avg,
            value.pressurization_pressure,
        );
        value.fuel_tank_pressure_1 = update_if_finite(
            &mut self.fuel_tank_pressure_1_avg,
            value.fuel_tank_pressure_1,
        );
        value.fuel_tank_pressure_2 = update_if_finite(
            &mut self.fuel_tank_pressure_2_avg,
            value.fuel_tank_pressure_2,
        );
        
        let filtered = get_filtered_tank_p(value.fuel_tank_pressure_1, value.fuel_tank_pressure_2);

        STATE
            .pressurization_pressure
            .sender()
            .send(BarG(value.pressurization_pressure));
        STATE
            .fuel_tank_pressure
            .sender()
            .send(dp_fuel_control_board::FuelTankPressure {
                fuel_tank_pressure_sensor_1: BarG(value.fuel_tank_pressure_1),
                fuel_tank_pressure_sensor_2: BarG(value.fuel_tank_pressure_2),
                fuel_tank_pressure_filtered: BarG(filtered
                ),
            });
        
        STATE.dpr_pressure.sender().send(
            filtered
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

fn get_filtered_tank_p(p1: f32, p2: f32) -> f32 {
    if p1.is_infinite() && p1.is_sign_positive() || p2.is_infinite() && p2.is_sign_positive() {
        f32::INFINITY
    } else if p1.is_finite() && p2.is_finite() {
        f32::max(p1, p2)
    } else if p1.is_finite() {
        p1
    } else if p2.is_finite() {
        p2
    } else {
        f32::INFINITY
    }
}
