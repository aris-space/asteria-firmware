use crate::globals::STATE;
use datatypes::status::SensorStatus;
use datatypes::units::BarG;
use filters::GaussianMovingAverage;

// Telemetry-only filtering. The DPR control loop is deliberately fed the raw
// reduction so that it never sees the filter group delay.
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
        }
    }

    pub fn update(&mut self, value: FuelPressureMeasurementRaw) -> FuelPressureMeasurementRaw {
        let has_error = !value.pressurization_pressure.is_finite()
            || !value.fuel_tank_pressure_1.is_finite()
            || !value.fuel_tank_pressure_2.is_finite();

        // Control path. Reduced from the raw sensor values and published before any
        // filtering happens, so the DPR loop sees the freshest possible pressure.
        let dpr_control_pressure =
            get_filtered_tank_p(value.fuel_tank_pressure_1, value.fuel_tank_pressure_2);
        STATE.dpr_pressure.sender().send(dpr_control_pressure);

        // Telemetry path. Each channel is filtered independently; the reduction is
        // then applied to the filtered values so the reported tank pressure is
        // consistent with the reported individual sensors.
        let pressurization_pressure = update_if_finite(
            &mut self.pressurization_pressure_avg,
            value.pressurization_pressure,
        );
        let fuel_tank_pressure_1 = update_if_finite(
            &mut self.fuel_tank_pressure_1_avg,
            value.fuel_tank_pressure_1,
        );
        let fuel_tank_pressure_2 = update_if_finite(
            &mut self.fuel_tank_pressure_2_avg,
            value.fuel_tank_pressure_2,
        );
        let fuel_tank_pressure_filtered =
            get_filtered_tank_p(fuel_tank_pressure_1, fuel_tank_pressure_2);

        STATE
            .pressurization_pressure
            .sender()
            .send(BarG(pressurization_pressure));
        STATE
            .fuel_tank_pressure_sensor_1
            .sender()
            .send(BarG(fuel_tank_pressure_1));
        STATE
            .fuel_tank_pressure_sensor_2
            .sender()
            .send(BarG(fuel_tank_pressure_2));
        STATE
            .fuel_tank_pressure_filtered
            .sender()
            .send(BarG(fuel_tank_pressure_filtered));

        STATE.pressure_bus_status.sender().send(if has_error {
            SensorStatus::Offline
        } else {
            SensorStatus::Online
        });

        value
    }
}

/// Feeds `value` into `avg` only when it is a real measurement. Sentinel values
/// (`NaN`, `+INF`, `-INF`) are passed straight through so that a sensor fault
/// cannot poison the filter window.
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
