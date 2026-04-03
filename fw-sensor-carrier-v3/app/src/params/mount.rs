use nalgebra::Matrix3;
use serde::{Deserialize, Serialize};

use super::Config;
use crate::sensors::{IMU_COUNT, ImuId};
use crate::storage::KeyStorage;

const BYTES: usize = 64;

#[derive(Clone, Serialize, Deserialize)]
pub struct ImuMount {
    pub rotation: [[f32; 3]; 3],
}

impl ImuMount {
    pub const DEFAULT: Self = Self {
        // Chip X/Z axes inverted relative to body frame (180 degrees around Y).
        rotation: [[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]],
    };

    pub fn rotation_matrix(&self) -> Matrix3<f32> {
        let r = &self.rotation;
        Matrix3::new(
            r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2],
        )
    }
}

pub static IMU_MOUNTS: [Config<ImuMount, BYTES>; IMU_COUNT] = [
    Config::new("imu_mount_0", ImuMount::DEFAULT),
    Config::new("imu_mount_1", ImuMount::DEFAULT),
];

pub async fn load_all(backend: &mut impl KeyStorage) {
    for config in &IMU_MOUNTS {
        config.load_from_backend(backend).await;
    }
}

pub fn imu_mount(id: ImuId) -> &'static Config<ImuMount, BYTES> {
    &IMU_MOUNTS[id.index()]
}
