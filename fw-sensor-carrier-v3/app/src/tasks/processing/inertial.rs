use defmt::info;
use embassy_futures::join::join;
use embassy_time::Timer;
use lsm6dso32::types::{Acceleration, AngularRate};
use nalgebra::{Matrix3, Vector3};

use crate::measurements::{ImuData, ImuSample, Timestamped};
use crate::params::calibration;
use crate::params::mount;
use crate::params::{Config, ConfigSnapshot, ConfigSource};
use crate::sensors::{IMU_0, IMU_1, ImuId};
use crate::signals;
use crate::storage;

/// Delay before the stationary calibration starts, to let the gyro's turn-on
/// bias transient settle as the die warms up.
const CAL_WARMUP_SECS: u64 = 30;

/// Samples averaged for the boot-time stationary calibration.
const CAL_SAMPLES: u32 = 32 * 1024;

/// Expected specific-force reading in the body (NED)
const EXPECTED_ACCEL_BODY_G: Vector3<f32> = Vector3::new(0.0, 0.0, -1.0);

struct ImuConfig {
    full_rot: Matrix3<f32>,
    accel_bias: Vector3<f32>,
    gyro_bias: Vector3<f32>,
}

impl ImuConfig {
    fn process(&self, sample: &ImuSample) -> ImuSample {
        let a = &sample.data.value.accel;
        let g = &sample.data.value.gyro;
        let accel = self.full_rot * (Vector3::new(a.x, a.y, a.z) - self.accel_bias);
        let gyro = self.full_rot * (Vector3::new(g.x, g.y, g.z) - self.gyro_bias);

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

/// Mount rotation composed with any stored fine-rotation correction. The
/// runtime stationary cal still owns the bias — only the rotation comes
/// from storage.
fn full_rot_for(id: ImuId) -> Matrix3<f32> {
    let mount = startup_value(mount::imu_mount(id)).rotation_matrix();
    let cal = startup_value(calibration::imu_cal(id));
    if cal.valid {
        cal.fine_rot_matrix() * mount
    } else {
        mount
    }
}

async fn run(id: ImuId) -> ! {
    let mut sub = signals::IMU_CHANNELS[id.index()].subscriber().unwrap();
    let full_rot = full_rot_for(id);

    Timer::after_secs(CAL_WARMUP_SECS).await;

    // Stationary calibration. Assumes the unit is at rest with attitude
    // identity in NED.
    let mut accel_sum = Vector3::<f32>::zeros();
    let mut gyro_sum = Vector3::<f32>::zeros();
    for _ in 0..CAL_SAMPLES {
        let sample = sub.next_message_pure().await;
        let a = &sample.data.value.accel;
        let g = &sample.data.value.gyro;
        accel_sum += Vector3::new(a.x, a.y, a.z);
        gyro_sum += Vector3::new(g.x, g.y, g.z);
    }
    let n = CAL_SAMPLES as f32;
    let expected_accel_raw = full_rot.transpose() * EXPECTED_ACCEL_BODY_G;
    let cfg = ImuConfig {
        full_rot,
        accel_bias: accel_sum / n - expected_accel_raw,
        gyro_bias: gyro_sum / n,
    };
    info!(
        "IMU {} cal: accel_bias=[{}, {}, {}] g, gyro_bias=[{}, {}, {}] dps",
        id.index(),
        cfg.accel_bias.x,
        cfg.accel_bias.y,
        cfg.accel_bias.z,
        cfg.gyro_bias.x,
        cfg.gyro_bias.y,
        cfg.gyro_bias.z,
    );

    loop {
        let sample = sub.next_message_pure().await;
        signals::submit_inertial_sample(cfg.process(&sample));
    }
}

#[embassy_executor::task]
pub async fn task() -> ! {
    storage::CONFIG_READY.wait().await;
    join(run(IMU_0), run(IMU_1)).await;
    unreachable!("processing tasks should never end");
}
