use datatypes::units::BarG;

// 4-20 mA through 150 Ω shunt → 0.6 V … 3.0 V
pub const V_MIN: f32 = 0.6;
pub const V_MAX: f32 = 3.0;

// Sensor pressure range [bar] — Keller sensor datasheet
pub const P_MIN_BAR: f32 = 0.0;
pub const P_MAX_BAR: f32 = 200.0;

// VREF used on the board
pub const VREF: f32 = 3.3;

pub const FILTER_WINDOW: usize = 10;
pub const FILTER_MEAN: f32 = 3.0;
pub const FILTER_SIGMA: f32 = 9.0;

#[derive(Clone, Copy)]
pub struct FuelTankPressureMeasurement {
    pub fuel_tank_pressure_1: BarG,
    pub fuel_tank_pressure_2: BarG,
    pub dpr_pressure: BarG,
}

pub fn raw_to_bar(raw: u16) -> f32 {
    let voltage = raw as f32 * VREF / 4095.0;
    P_MIN_BAR + (P_MAX_BAR - P_MIN_BAR) / (V_MAX - V_MIN) * (voltage - V_MIN)
}

/// Returns the best available tank pressure from the two redundant sensors.
/// Prefers the higher reading; falls back to whichever sensor is valid if one is NaN.
pub fn get_filtered_tank_p(p1: f32, p2: f32) -> f32 {
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
