use crate::globals::STATE;
use datatypes::status::SensorStatus;
use datatypes::units::BarG;

#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OxidizerPressureMeasurementRaw {
    pub oxidizer_tank_pressure_1: f32,
    pub oxidizer_tank_pressure_2: f32,
    pub oxidizer_tank_differential_pressure: f32,
}

pub struct OxidizerPressureDriver;

impl OxidizerPressureDriver {
    pub fn new() -> Self {
        Self
    }

    pub fn update(
        &mut self,
        value: OxidizerPressureMeasurementRaw,
    ) -> OxidizerPressureMeasurementRaw {
        let has_error = !value.oxidizer_tank_pressure_1.is_finite()
            || !value.oxidizer_tank_pressure_2.is_finite()
            || !value.oxidizer_tank_differential_pressure.is_finite();
        
        let oxidizer_tank_pressure_pid = get_filtered_tank_p(value.oxidizer_tank_pressure_1, value.oxidizer_tank_pressure_2);

        STATE
            .oxidizer_tank_pressure_sensor_1
            .sender()
            .send(BarG(value.oxidizer_tank_pressure_1));
        STATE
            .oxidizer_tank_pressure_sensor_2
            .sender()
            .send(BarG(value.oxidizer_tank_pressure_2));
        STATE
            .oxidizer_tank_differential_pressure
            .sender()
            .send(BarG(value.oxidizer_tank_differential_pressure));

        STATE.pressure_bus_status.sender().send(if has_error {
            SensorStatus::Offline
        } else {
            SensorStatus::Online
        });
        STATE
            .dpr_pressure
            .sender()
            .send(oxidizer_tank_pressure_pid);

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
