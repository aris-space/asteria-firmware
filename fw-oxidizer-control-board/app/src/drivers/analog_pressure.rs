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
pub struct OxidizerPressureMeasurementRaw {
    pub oxidizer_tank_pressure_1: f32,
    pub oxidizer_tank_pressure_2: f32,
    pub oxidizer_tank_differential_pressure: f32,
}

pub struct OxidizerPressureDriver {
    oxidizer_tank_pressure_1_avg: GaussianMovingAverage<CAN_FILTER_WINDOW>,
    oxidizer_tank_pressure_2_avg: GaussianMovingAverage<CAN_FILTER_WINDOW>,
    oxidizer_tank_differential_pressure_avg: GaussianMovingAverage<CAN_FILTER_WINDOW>,
}

impl OxidizerPressureDriver {
    pub fn new() -> Self {
        Self {
            oxidizer_tank_pressure_1_avg: GaussianMovingAverage::new(
                CAN_FILTER_SIGMA,
                CAN_FILTER_MEAN,
            ),
            oxidizer_tank_pressure_2_avg: GaussianMovingAverage::new(
                CAN_FILTER_SIGMA,
                CAN_FILTER_MEAN,
            ),
            oxidizer_tank_differential_pressure_avg: GaussianMovingAverage::new(
                CAN_FILTER_SIGMA,
                CAN_FILTER_MEAN,
            ),
        }
    }

    pub fn update(
        &mut self,
        value: OxidizerPressureMeasurementRaw,
    ) -> OxidizerPressureMeasurementRaw {
        let has_error = !value.oxidizer_tank_pressure_1.is_finite()
            || !value.oxidizer_tank_pressure_2.is_finite()
            || !value.oxidizer_tank_differential_pressure.is_finite();

        // Control path. Reduced from the raw sensor values and published before any
        // filtering happens, so the DPR loop sees the freshest possible pressure.
        let dpr_control_pressure = get_control_pressure(
            value.oxidizer_tank_pressure_1,
            value.oxidizer_tank_pressure_2,
        );
        STATE.dpr_pressure.sender().send(dpr_control_pressure);

        // Telemetry path. Each channel is filtered independently.
        let oxidizer_tank_pressure_1 = update_if_finite(
            &mut self.oxidizer_tank_pressure_1_avg,
            value.oxidizer_tank_pressure_1,
        );
        let oxidizer_tank_pressure_2 = update_if_finite(
            &mut self.oxidizer_tank_pressure_2_avg,
            value.oxidizer_tank_pressure_2,
        );
        let oxidizer_tank_differential_pressure = update_if_finite(
            &mut self.oxidizer_tank_differential_pressure_avg,
            value.oxidizer_tank_differential_pressure,
        );

        STATE
            .oxidizer_tank_pressure_sensor_1
            .sender()
            .send(BarG(oxidizer_tank_pressure_1));
        STATE
            .oxidizer_tank_pressure_sensor_2
            .sender()
            .send(BarG(oxidizer_tank_pressure_2));
        STATE
            .oxidizer_tank_differential_pressure
            .sender()
            .send(BarG(oxidizer_tank_differential_pressure));

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

/// Reduces the two redundant tank pressure sensors to the single value used as
/// the DPR control input.
fn get_control_pressure(p1: f32, p2: f32) -> f32 {
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
