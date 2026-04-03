use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use super::Config;
use crate::sensors::{IMU_COUNT, ImuId};
use crate::storage::KeyStorage;

const BYTES: usize = 128;

#[derive(Clone, Serialize, Deserialize)]
pub struct ImuCalib {
    pub gyr_bias: Vector3<f32>,
    pub fine_rot: Matrix3<f32>,
    pub valid: bool,
}

fn default_imu_calib() -> ImuCalib {
    ImuCalib {
        gyr_bias: Vector3::zeros(),
        fine_rot: Matrix3::identity(),
        valid: false,
    }
}

pub static IMU_CALS: [Config<ImuCalib, BYTES>; IMU_COUNT] = [
    Config::new("imu_cal_0", default_imu_calib),
    Config::new("imu_cal_1", default_imu_calib),
];

pub async fn load_all(backend: &mut impl KeyStorage) {
    for config in &IMU_CALS {
        config.load_from_backend(backend).await;
    }
}

pub fn imu_cal(id: ImuId) -> &'static Config<ImuCalib, BYTES> {
    &IMU_CALS[id.index()]
}
