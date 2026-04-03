use embassy_futures::join::join;
use lsm6dso32::types::{Acceleration, AngularRate};
use nalgebra::{Matrix3, Vector3};

use crate::measurements::{ImuData, ImuSample, Timestamped};
use crate::params::calibration::{self, ImuCalib};
use crate::params::mount::{self, ImuMount};
use crate::sensors::{IMU_0, IMU_1, ImuId};
use crate::signals;
use crate::tasks::storage;

struct ImuConfig {
    full_rot: Matrix3<f32>,
    bias: Vector3<f32>,
}

impl ImuConfig {
    fn new(mount: &ImuMount, cal: &ImuCalib) -> Self {
        let mount = mount.rotation_matrix();
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
            data: Timestamped::at(
                sample.data.ts,
                ImuData {
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
            ),
        }
    }
}

async fn run(id: ImuId) -> ! {
    let mut sub = signals::IMU_CHANNELS[id.index()].subscriber().unwrap();
    let mut calib_updates = calibration::imu_cal(id).receiver().unwrap();
    let mut mount_updates = mount::imu_mount(id).receiver().unwrap();

    let mut mount = mount_updates.get().await;
    let mut calib = calib_updates.get().await;
    let mut cfg = ImuConfig::new(&mount, &calib);

    loop {
        let mut changed = false;

        if let Some(next_mount) = mount_updates.try_changed() {
            mount = next_mount;
            changed = true;
        }

        if let Some(next_calib) = calib_updates.try_changed() {
            calib = next_calib;
            changed = true;
        }

        if changed {
            cfg = ImuConfig::new(&mount, &calib);
        }

        let sample = sub.next_message_pure().await;
        signals::submit_inertial_sample(cfg.process(&sample));
    }
}

#[embassy_executor::task]
pub async fn task() -> ! {
    storage::READY.wait().await;

    join(run(IMU_0), run(IMU_1)).await;
    unreachable!("processing tasks should never end");
}
