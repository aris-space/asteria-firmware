use nalgebra::Matrix3;
use serde::{Deserialize, Serialize};

use super::{Config, RegistryEntry};
use crate::sensors::{IMU_COUNT, ImuId};
use crate::tasks::storage;

const IMU_MOUNT_BYTES: usize = 64;

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

pub static IMU_MOUNTS: [Config<ImuMount, IMU_MOUNT_BYTES>; IMU_COUNT] = [
    Config::new("imu_mount_0", ImuMount::DEFAULT),
    Config::new("imu_mount_1", ImuMount::DEFAULT),
];

fn load_imu_mount_0(fs: &storage::Fs) {
    IMU_MOUNTS[0].load_from_fs(fs);
}

fn save_imu_mount_0(fs: &storage::Fs) {
    IMU_MOUNTS[0].save_to_fs(fs);
}

fn load_imu_mount_1(fs: &storage::Fs) {
    IMU_MOUNTS[1].load_from_fs(fs);
}

fn save_imu_mount_1(fs: &storage::Fs) {
    IMU_MOUNTS[1].save_to_fs(fs);
}

pub static REGISTRY: [RegistryEntry; IMU_COUNT] = [
    RegistryEntry::new(load_imu_mount_0, save_imu_mount_0),
    RegistryEntry::new(load_imu_mount_1, save_imu_mount_1),
];

pub fn imu_mount(id: ImuId) -> &'static Config<ImuMount, IMU_MOUNT_BYTES> {
    &IMU_MOUNTS[id.index()]
}
