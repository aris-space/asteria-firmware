//! Calibrated sensor observations accepted by estimator banks.

/// One calibrated IMU observation and its source.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuInput {
    /// Time represented by the physical measurement on the estimator clock, in microseconds.
    /// Apply clock-offset and known latency calibration before constructing this input.
    pub timestamp_us: u64,
    /// Zero-based index of the source sensor.
    pub sensor_index: usize,
    /// Specific force in body axes, in m/s².
    pub acceleration_body_mps2: [f32; 3],
    /// Angular rate in body axes, in rad/s.
    pub angular_rate_body_rad_s: [f32; 3],
}

/// One barometer observation in the forms consumed by the two current filters.
///
/// The caller supplies both the calibrated height and the pressure reading. This keeps each
/// filter's existing measurement model unchanged.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BarometerInput {
    /// Time represented by the physical measurement on the estimator clock, in microseconds.
    /// Apply clock-offset and known latency calibration before constructing this input.
    pub timestamp_us: u64,
    /// Zero-based index of the source sensor.
    pub sensor_index: usize,
    /// Calibrated up-positive height in metres, used by SEF Light.
    pub height_m: f32,
    /// Standard deviation of `height_m`, in metres.
    pub height_std_m: f32,
    /// Measured atmospheric pressure in hPa, used by SEF Large.
    pub pressure_hpa: f32,
    /// Preflight reference pressure for this barometer, in hPa.
    pub reference_pressure_hpa: f32,
    /// Standard deviation of `pressure_hpa`, in hPa.
    pub pressure_std_hpa: f32,
}

/// One selected receiver's GNSS solution in both vertical and full-navigation frames.
///
/// Select a receiver before passing its fix to the estimator. Each call represents one physical
/// measurement; passing both receivers' fixes at the same time would fuse both measurements.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssInput {
    /// Time represented by the physical measurement on the estimator clock, in microseconds.
    /// Apply clock-offset and known latency calibration before constructing this input.
    pub timestamp_us: u64,
    /// Zero-based index of the source receiver.
    pub sensor_index: usize,
    /// Up-positive height in the caller's absolute altitude frame, in metres.
    pub height_m: f32,
    /// Up-positive vertical velocity in m/s.
    pub vertical_velocity_mps: f32,
    /// Standard deviation of `height_m`, in metres.
    pub height_std_m: f32,
    /// Standard deviation of `vertical_velocity_mps`, in m/s.
    pub vertical_velocity_std_mps: f32,
    /// Position relative to the navigation reference, in north-east-down order and metres.
    pub position_ned_m: [f32; 3],
    /// Velocity in north-east-down order and m/s.
    pub velocity_ned_mps: [f32; 3],
    /// Per-axis standard deviations of `position_ned_m`, in metres.
    pub position_std_m: [f32; 3],
    /// Per-axis standard deviations of `velocity_ned_mps`, in m/s.
    pub velocity_std_mps: [f32; 3],
}

/// One calibrated magnetic-field observation.
///
/// SEF Light associates `sensor_index` with an IMU attitude chain. SEF Large associates it with
/// a magnetometer branch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagnetometerInput {
    /// Time represented by the physical measurement on the estimator clock, in microseconds.
    /// Apply clock-offset and known latency calibration before constructing this input.
    pub timestamp_us: u64,
    /// Zero-based index of the source sensor.
    pub sensor_index: usize,
    /// Calibrated magnetic-field vector in body axes.
    pub field_body: [f32; 3],
    /// Expected local magnetic-field vector in north-east-down axes, used by SEF Large.
    pub reference_field_ned: [f32; 3],
    /// Standard deviation of each normalized magnetic direction component, used by SEF Large.
    pub direction_noise_std: f32,
}
