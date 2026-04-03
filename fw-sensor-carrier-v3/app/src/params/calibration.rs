use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use super::{Config, load_all_configs};
use crate::sensors::{IMU_COUNT, ImuId};
use crate::storage::KeyStorage;

const BYTES: usize = 128;
type ImuCalParam = Config<ImuCalib, BYTES>;

#[derive(Clone, Serialize, Deserialize)]
pub struct ImuCalib {
    pub gyr_bias: Vector3<f32>,
    pub fine_rot: Matrix3<f32>,
    pub valid: bool,
}

impl Default for ImuCalib {
    fn default() -> Self {
        Self {
            gyr_bias: Vector3::zeros(),
            fine_rot: Matrix3::identity(),
            valid: false,
        }
    }
}

pub static IMU_CALS: [ImuCalParam; IMU_COUNT] =
    [Config::new("imu_cal_0"), Config::new("imu_cal_1")];

pub async fn load_all(backend: &mut impl KeyStorage) {
    load_all_configs(backend, &IMU_CALS).await;
}

pub fn imu_cal(id: ImuId) -> &'static ImuCalParam {
    &IMU_CALS[id.index()]
}
