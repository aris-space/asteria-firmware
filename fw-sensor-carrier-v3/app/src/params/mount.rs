use nalgebra::Matrix3;
use serde::{Deserialize, Serialize};

use super::{Config, load_all_configs};
use crate::sensors::{IMU_COUNT, ImuId};
use crate::storage::KeyStorage;

const BYTES: usize = 64;
type ImuMountParam = Config<ImuMount, BYTES>;

#[derive(Clone, Serialize, Deserialize)]
pub struct ImuMount {
    pub rotation: Matrix3<f32>,
}

impl Default for ImuMount {
    fn default() -> Self {
        Self {
            // Chip X/Z axes inverted relative to body frame (180 degrees around Y).
            rotation: Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0),
        }
    }
}

pub static IMU_MOUNTS: [ImuMountParam; IMU_COUNT] =
    [Config::new("imu_mount_0"), Config::new("imu_mount_1")];

pub async fn load_all(backend: &mut impl KeyStorage) {
    load_all_configs(backend, &IMU_MOUNTS).await;
}

pub fn imu_mount(id: ImuId) -> &'static ImuMountParam {
    &IMU_MOUNTS[id.index()]
}
