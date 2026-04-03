use embassy_futures::join::join;
use lsm6dso32::types::{Acceleration, AngularRate};
use nalgebra::{Matrix3, Vector3};

use crate::measurements::{ImuData, ImuSample, Timestamped};
use crate::params::calibration::{self, ImuCalib};
use crate::params::mount::{self, ImuMount};
use crate::params::{Config, ConfigSnapshot, ConfigSource};
use crate::sensors::{IMU_0, IMU_1, ImuId};
use crate::signals;
use crate::storage;

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

fn startup_value<T: Clone, const BYTES: usize, const WATCHERS: usize>(
    config: &'static Config<T, BYTES, WATCHERS>,
) -> T {
    match config.snapshot() {
        ConfigSnapshot {
            source: ConfigSource::Persisted | ConfigSource::Runtime,
            value: Some(value),
        } => value,
        ConfigSnapshot {
            source: ConfigSource::Missing | ConfigSource::Invalid | ConfigSource::Unavailable,
            ..
        } => config.default_value(),
        ConfigSnapshot { value: None, .. } => config.default_value(),
    }
}

fn config_for(id: ImuId) -> ImuConfig {
    let mount = startup_value(mount::imu_mount(id));
    let calib = startup_value(calibration::imu_cal(id));
    ImuConfig::new(&mount, &calib)
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
    storage::CONFIG_READY.wait().await;

    let cfg0 = config_for(IMU_0);
    let cfg1 = config_for(IMU_1);

    join(run(IMU_0, &cfg0), run(IMU_1, &cfg1)).await;
    unreachable!("processing tasks should never end");
}
