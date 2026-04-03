use nalgebra::Matrix3;

use crate::sensors::{IMU_COUNT, ImuId, MAGNETOMETER_COUNT, MagnetometerId};

pub const IMU_MOUNTS: [Matrix3<f32>; IMU_COUNT] = [
    // IMU_0: chip X/Z axes inverted relative to body frame (180° around Y)
    Matrix3::new(
        -1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, -1.0,
    ),
    // IMU_1: same orientation as IMU_0 on this board
    Matrix3::new(
        -1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, -1.0,
    ),
];

pub const MAG_MOUNTS: [Matrix3<f32>; MAGNETOMETER_COUNT] = [
    // MAG_0: all axes inverted
    Matrix3::new(
        -1.0,  0.0,  0.0,
         0.0, -1.0,  0.0,
         0.0,  0.0, -1.0,
    ),
    // MAG_1: same as MAG_0 on this board
    Matrix3::new(
        -1.0,  0.0,  0.0,
         0.0, -1.0,  0.0,
         0.0,  0.0, -1.0,
    ),
];

pub fn imu_mount(id: ImuId) -> &'static Matrix3<f32> {
    &IMU_MOUNTS[id.index()]
}

pub fn mag_mount(id: MagnetometerId) -> &'static Matrix3<f32> {
    &MAG_MOUNTS[id.index()]
}

