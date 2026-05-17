use crate::globals::STATE;
use datatypes::status::SensorStatus;
use datatypes::units::BarG;
use filters::GaussianMovingAverage;

const PID_FILTER_WINDOW: usize = 7;
const PID_FILTER_MEAN: f32 = 6.0;
const PID_FILTER_SIGMA: f32 = 2.0;

const CAN_FILTER_WINDOW: usize = 25;
const CAN_FILTER_MEAN: f32 = 24.0;
const CAN_FILTER_SIGMA: f32 = 7.0;

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FuelPressureMeasurementRaw {
    pub pressurization_pressure: f32,
    pub fuel_tank_pressure_1: f32,
    pub fuel_tank_pressure_2: f32,
}

pub struct FuelPressureDriver {
    pressurization_pressure_avg: GaussianMovingAverage<CAN_FILTER_WINDOW>,
    fuel_tank_pressure_1_avg: GaussianMovingAverage<CAN_FILTER_WINDOW>,
    fuel_tank_pressure_2_avg: GaussianMovingAverage<CAN_FILTER_WINDOW>,
    fuel_tank_pressure_pid_avg: GaussianMovingAverage<PID_FILTER_WINDOW>,
}

impl FuelPressureDriver {
    pub fn new() -> Self {
        Self {
            pressurization_pressure_avg: GaussianMovingAverage::new(
                CAN_FILTER_SIGMA,
                CAN_FILTER_MEAN,
            ),
            fuel_tank_pressure_1_avg: GaussianMovingAverage::new(CAN_FILTER_SIGMA, CAN_FILTER_MEAN),
            fuel_tank_pressure_2_avg: GaussianMovingAverage::new(CAN_FILTER_SIGMA, CAN_FILTER_MEAN),
            fuel_tank_pressure_pid_avg: GaussianMovingAverage::new(
                PID_FILTER_SIGMA,
                PID_FILTER_MEAN,
            ),
        }
    }

    pub fn update(&mut self, mut value: FuelPressureMeasurementRaw) -> FuelPressureMeasurementRaw {
        let has_error = !value.pressurization_pressure.is_finite()
            || !value.fuel_tank_pressure_1.is_finite()
            || !value.fuel_tank_pressure_2.is_finite();

        let fuel_tank_pressure_pid = update_if_finite(
            &mut self.fuel_tank_pressure_pid_avg,
            get_filtered_tank_p(value.fuel_tank_pressure_1, value.fuel_tank_pressure_2),
        );

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

        STATE
            .pressurization_pressure
            .sender()
            .send(BarG(value.pressurization_pressure));
        STATE
            .fuel_tank_pressure_sensor_1
            .sender()
            .send(BarG(value.fuel_tank_pressure_1));
        STATE
            .fuel_tank_pressure_sensor_2
            .sender()
            .send(BarG(value.fuel_tank_pressure_2));
        STATE
            .fuel_tank_pressure_filtered
            .sender()
            .send(BarG(fuel_tank_pressure_pid));
        STATE.dpr_pressure.sender().send(fuel_tank_pressure_pid);

        STATE.pressure_bus_status.sender().send(if has_error {
            SensorStatus::Offline
        } else {
            SensorStatus::Online
        });

        value
    }
}

fn update_if_finite<const SIZE: usize>(avg: &mut GaussianMovingAverage<SIZE>, value: f32) -> f32 {
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
