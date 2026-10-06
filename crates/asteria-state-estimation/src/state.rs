//! Navigation solution reported by every estimator.

/// The selected navigation solution in the north-east-down frame of a [`GeodeticReference`].
///
/// Every estimator fills every field. A quantity an estimator does not observe carries the
/// uncertainty it actually has, which may be infinite.
///
/// [`GeodeticReference`]: crate::GeodeticReference
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavigationState {
    /// Newest IMU sample the state is propagated to, in microseconds on the estimator clock.
    /// Zero before the first IMU sample.
    pub time_us: u64,
    /// Position relative to the reference, in north-east-down order and metres.
    pub position_ned_m: [f32; 3],
    /// Per-axis standard deviations of `position_ned_m`, in metres.
    pub position_std_ned_m: [f32; 3],
    /// Velocity in north-east-down order and m/s.
    pub velocity_ned_mps: [f32; 3],
    /// Per-axis standard deviations of `velocity_ned_mps`, in m/s.
    pub velocity_std_ned_mps: [f32; 3],
    /// Scalar-first unit quaternion rotating body axes into north-east-down axes.
    pub orientation_body_to_ned_wxyz: [f32; 4],
    /// Newest angular rate of the selected IMU in body axes, with any estimated gyroscope bias
    /// removed, in rad/s.
    pub angular_rate_body_rad_s: [f32; 3],
    /// Newest specific force of the selected IMU in body axes, with any estimated accelerometer
    /// bias removed, in m/s².
    pub specific_force_body_mps2: [f32; 3],
}
