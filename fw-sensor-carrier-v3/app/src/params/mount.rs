use nalgebra::Matrix3;

use crate::sensors::{IMU_COUNT, ImuId};

pub const IMU_MOUNTS: [Matrix3<f32>; IMU_COUNT] = [
    // IMU_0: chip X/Z axes inverted relative to body frame (180° around Y)
    Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0),
    // IMU_1: same orientation as IMU_0 on this board
    Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0),
];

pub fn imu_mount(id: ImuId) -> &'static Matrix3<f32> {
    &IMU_MOUNTS[id.index()]
}
