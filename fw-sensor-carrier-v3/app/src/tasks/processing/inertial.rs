use embassy_futures::join::join;
use lsm6dso32::types::{Acceleration, AngularRate};
use nalgebra::{Matrix3, Vector3};

use crate::measurements::{ImuData, ImuSample, Timestamped};
use crate::params::calibration::{self, ImuCalib};
use crate::params::mount;
use crate::sensors::{IMU_0, IMU_1, ImuId};
use crate::signals;
use crate::tasks::storage;

struct ImuConfig {
    full_rot: Matrix3<f32>,
    bias: Vector3<f32>,
}

impl ImuConfig {
    fn new(id: ImuId, cal: &ImuCalib) -> Self {
        let mount = *mount::imu_mount(id);
        if cal.valid {
            Self {
                full_rot: cal.fine_rot_matrix() * mount,
                bias: cal.bias_vector(),
            }
        } else {
            Self {
                full_rot: mount,
                bias: Vector3::zeros(),
            }
        }
    }

    fn process(&self, sample: &ImuSample) -> ImuSample {
        let a = &sample.data.value.accel;
        let g = &sample.data.value.gyro;
        let accel = self.full_rot * Vector3::new(a.x, a.y, a.z);
        let gyro = self.full_rot * (Vector3::new(g.x, g.y, g.z) - self.bias);

        ImuSample {
            sensor_id: sample.sensor_id,
            data: Timestamped {
                ts: sample.data.ts,
                value: ImuData {
                    accel: Acceleration {
                        x: accel.x,
                        y: accel.y,
                        z: accel.z,
                    },
                    gyro: AngularRate {
                        x: gyro.x,
                        y: gyro.y,
                        z: gyro.z,
                    },
                },
            },
        }
    }
}

async fn run(id: ImuId, cfg: &ImuConfig) -> ! {
    let mut sub = signals::IMU_CHANNELS[id.index()].subscriber().unwrap();
    loop {
        let sample = sub.next_message_pure().await;
        signals::submit_inertial_sample(cfg.process(&sample));
    }
}

#[embassy_executor::task]
pub async fn task() -> ! {
    storage::READY.wait().await;

    let cfg0 = ImuConfig::new(IMU_0, &calibration::imu_cal(IMU_0).snapshot());
    let cfg1 = ImuConfig::new(IMU_1, &calibration::imu_cal(IMU_1).snapshot());

    join(run(IMU_0, &cfg0), run(IMU_1, &cfg1)).await;
    unreachable!("processing tasks should never end");
}
