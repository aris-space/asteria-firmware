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

        value
    }
}
