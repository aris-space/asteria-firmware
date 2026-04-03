use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use super::{Config, ConfigBackend, RegistryEntry};
use crate::sensors::{IMU_COUNT, ImuId};

const IMU_CAL_BYTES: usize = 128;

#[derive(Clone, Serialize, Deserialize)]
pub struct ImuCalib {
    pub gyr_bias: [f32; 3],
    pub fine_rot: [[f32; 3]; 3],
    pub valid: bool,
}

impl ImuCalib {
    pub const DEFAULT: Self = Self {
        gyr_bias: [0.0; 3],
        fine_rot: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        valid: false,
    };

    pub fn fine_rot_matrix(&self) -> Matrix3<f32> {
        let r = &self.fine_rot;
        Matrix3::new(
            r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2],
        )
    }

    pub fn bias_vector(&self) -> Vector3<f32> {
        Vector3::from_column_slice(&self.gyr_bias)
    }
}

pub static IMU_CAL: [Config<ImuCalib, IMU_CAL_BYTES>; IMU_COUNT] = [
    Config::new("imu_cal_0", ImuCalib::DEFAULT),
    Config::new("imu_cal_1", ImuCalib::DEFAULT),
];

fn load_imu_cal_0(backend: &dyn ConfigBackend) {
    IMU_CAL[0].load_from_backend(backend);
}

fn load_imu_cal_1(backend: &dyn ConfigBackend) {
    IMU_CAL[1].load_from_backend(backend);
}

pub static REGISTRY: [RegistryEntry; IMU_COUNT] = [
    RegistryEntry::new(load_imu_cal_0),
    RegistryEntry::new(load_imu_cal_1),
];

pub fn imu_cal(id: ImuId) -> &'static Config<ImuCalib, IMU_CAL_BYTES> {
    &IMU_CAL[id.index()]
}
