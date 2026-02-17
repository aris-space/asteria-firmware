pub(crate) mod dpr;
pub(crate) mod valves;

pub(crate) const SAFETY_LIMIT_BARG: f32 = 55.0;
pub(crate) const CYCLE_TIME_MS: f32 = 16.67; // 16.67 ms cycle time for 60 Hz, matching the pressure acquisition rate
pub(crate) const KP: f32 = 1.0;
pub(crate) const KI: f32 = 0.0;
pub(crate) const KD: f32 = 8.0;
