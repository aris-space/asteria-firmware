use nalgebra::Matrix3;
use serde::{Deserialize, Serialize};

use super::Config;
use crate::sensors::{IMU_COUNT, ImuId};
use crate::storage::KeyStorage;

const BYTES: usize = 64;

#[derive(Clone, Serialize, Deserialize)]
pub struct ImuMount {
    pub rotation: Matrix3<f32>,
}

fn default_imu_mount() -> ImuMount {
    ImuMount {
        // Chip X/Z axes inverted relative to body frame (180 degrees around Y).
        rotation: Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0),
    }
}

pub static IMU_MOUNTS: [Config<ImuMount, BYTES>; IMU_COUNT] = [
    Config::new("imu_mount_0", default_imu_mount),
    Config::new("imu_mount_1", default_imu_mount),
];

pub async fn load_all(backend: &mut impl KeyStorage) {
    for config in &IMU_MOUNTS {
        config.load_from_backend(backend).await;
    }
}

pub fn imu_mount(id: ImuId) -> &'static Config<ImuMount, BYTES> {
    &IMU_MOUNTS[id.index()]
}
