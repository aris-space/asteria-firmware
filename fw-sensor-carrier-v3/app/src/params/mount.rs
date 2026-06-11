use nalgebra::Matrix3;
use serde::{Deserialize, Serialize};

use super::{Config, RegistryEntry, indexed_loaders, registry_entries};
use crate::sensors::{IMU_COUNT, ImuId};

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

    pub const EVALUATION: Self = Self {
        // Evaluation board mount: chip X/Y swapped and Y inverted relative to body frame
        rotation: [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
    };

    pub fn rotation_matrix(&self) -> Matrix3<f32> {
        let r = &self.rotation;
        Matrix3::new(
            r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2],
        )
    }
}

pub static IMU_MOUNTS: [Config<ImuMount, BYTES>; IMU_COUNT] = [
    Config::new("imu_mount_0", ImuMount::EVALUATION),
    Config::new("imu_mount_1", ImuMount::EVALUATION),
];

indexed_loaders!(
    IMU_MOUNTS,
    load_imu_mount_0 => 0,
    load_imu_mount_1 => 1,
);

pub static REGISTRY: [RegistryEntry; IMU_COUNT] =
    registry_entries![load_imu_mount_0, load_imu_mount_1];

pub fn imu_mount(id: ImuId) -> &'static Config<ImuMount, BYTES> {
    &IMU_MOUNTS[id.index()]
}
