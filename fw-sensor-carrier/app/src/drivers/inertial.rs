use crate::drivers::magnetic_field;
use crate::drivers::magnetic_field::MagMeasurement;
use crate::sensors::SensorId;
use crate::sensors::imu::{IMU_ODR_HZ, IMU_TARGET_DT};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_sync::pubsub::{ImmediatePublisher, PubSubChannel};
use embassy_sync::watch;
use embassy_sync::watch::{Sender, Watch};
use embassy_time::{Duration, Instant};
use embedded_utils::ExtendTime;
use embedded_utils::fmt::*;
use hermes_can::messages::sensor_data::ImuData;
use imu_fusion::{Fusion, FusionAhrsSettings, FusionVector};
use lsm6dso32::{Acceleration, AngularRate};
use nalgebra::{Quaternion, UnitQuaternion, Vector3};

pub const CAP: usize = 10;
pub const PUB: usize = 0;
pub const SUB: usize = 2;
pub const WATCH: usize = 3;

pub static ORIENTATION_WATCH: Watch<ThreadModeRawMutex, UnitQuaternion<f32>, WATCH> = Watch::new();
pub static ORIENTATION_PUBSUB: PubSubChannel<
    ThreadModeRawMutex,
    UnitQuaternion<f32>,
    CAP,
    SUB,
    PUB,
> = PubSubChannel::new();

pub static INERTIAL_WATCH: Watch<ThreadModeRawMutex, ImuData, WATCH> = Watch::new();

pub static INERTIAL_PUBSUB: PubSubChannel<ThreadModeRawMutex, ImuData, CAP, SUB, PUB> =
    PubSubChannel::new();

pub static INERTIAL_DRIVER: OnceLock<InertialDriver> = OnceLock::new();

pub type ImuMeasurement = (Instant, AngularRate, Acceleration);

struct SharedData<'a> {
    magnetometer_watch:
        watch::Receiver<'a, ThreadModeRawMutex, MagMeasurement, { magnetic_field::WATCH }>,

    fusion: Fusion,
    prev_ts: Option<Instant>,

    timeout_selector: TimeoutSelector,
}

pub struct InertialDriver<'a> {
    orientation_pub: ImmediatePublisher<'a, ThreadModeRawMutex, UnitQuaternion<f32>, CAP, SUB, PUB>,
    orientation_watch: Sender<'a, ThreadModeRawMutex, UnitQuaternion<f32>, WATCH>,

    inertial_pub: ImmediatePublisher<'a, ThreadModeRawMutex, ImuData, CAP, SUB, PUB>,
    inertial_watch: Sender<'a, ThreadModeRawMutex, ImuData, WATCH>,

    shared: Mutex<CriticalSectionRawMutex, SharedData<'a>>,
}

impl<'a> InertialDriver<'a> {
    pub fn new(
        orientation_pub: ImmediatePublisher<
            'a,
            ThreadModeRawMutex,
            UnitQuaternion<f32>,
            CAP,
            SUB,
            PUB,
        >,
        orientation_watch: Sender<'a, ThreadModeRawMutex, UnitQuaternion<f32>, WATCH>,
        inertial_pub: ImmediatePublisher<'a, ThreadModeRawMutex, ImuData, CAP, SUB, PUB>,
        inertial_watch: Sender<'a, ThreadModeRawMutex, ImuData, WATCH>,
        magnetometer_watch: watch::Receiver<
            'a,
            ThreadModeRawMutex,
            MagMeasurement,
            { magnetic_field::WATCH },
        >,
    ) -> Self {
        Self {
            orientation_pub,
            orientation_watch,
            inertial_pub,
            inertial_watch,
            shared: Mutex::new(SharedData {
                magnetometer_watch,
                fusion: fusion_instance(),
                prev_ts: None,
                timeout_selector: TimeoutSelector::new(Duration::from_millis(100)),
            }),
        }
    }

    #[allow(dead_code)]
    pub async fn set_heading(&self, heading: f32) {
        info!("Setting heading to {:?}", heading);
        self.shared.lock().await.fusion.set_heading(heading);
    }

    /// Update the IMU data with a fresh measurement.
    pub async fn update_imu_realtime(
        &self,
        gyro: AngularRate,
        accel: Acceleration,
        ts: Instant,
        id: SensorId,
    ) {
        self.process_imu(gyro, accel, ts, id, true).await;
    }

    /// Update the IMU data with a possibly stale measurement (from backlog).
    pub async fn update_imu_stale(
        &self,
        gyro: AngularRate,
        accel: Acceleration,
        ts: Instant,
        id: SensorId,
    ) {
        self.process_imu(gyro, accel, ts, id, false).await;
    }

    async fn process_imu(
        &self,
        gyro: AngularRate,
        accel: Acceleration,
        ts: Instant,
        id: SensorId,
        realtime: bool,
    ) {
        if !matches!(id, SensorId::Imu1 | SensorId::Imu2) {
            error!("Invalid sensor id for IMU update: {:?}", id);
            return;
        }

        let fusion_quaternion = {
            let mut shared = self.shared.lock().await;

            if !shared.timeout_selector.accept(id, ts) {
                return;
            }

            let dt = shared.prev_ts.map_or(IMU_TARGET_DT, |p| {
                ts.saturating_duration_since(p).as_secs_f32()
            });

            let gyro_vec = FusionVector::new(gyro.x, gyro.y, gyro.z);
            let accel_vec = FusionVector::new(accel.x, accel.y, accel.z);
            let mag_vec = realtime
                .then(|| shared.magnetometer_watch.try_changed())
                .flatten();

            if let Some((_ts, mag)) = mag_vec {
                // update the fusion with the latest mag measurement
                shared.fusion.update_by_duration_seconds(
                    gyro_vec,
                    accel_vec,
                    FusionVector::new(mag.x_nt() as f32, mag.y_nt() as f32, mag.z_nt() as f32),
                    dt,
                );
            } else {
                // no mag available or not realtime, so we use the accel only
                shared
                    .fusion
                    .update_no_mag_by_duration_seconds(gyro_vec, accel_vec, dt);
            }

            shared.prev_ts = Some(ts);
            shared.fusion.quaternion()
        };

        let orientation_mag = UnitQuaternion::new_normalize(Quaternion::new(
            fusion_quaternion.w,
            fusion_quaternion.x,
            fusion_quaternion.y,
            fusion_quaternion.z,
        ));

        // Taken from https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#igrfwmm
        // Calculated for Gadmen Range, Switzerland
        // Model Used:  WMMHR-2025
        // Latitude:    46° 44' 59" N
        // Longitude:   8° 23' 11" E
        // Elevation:   1606.0 m Mean Sea Level
        //
        // 2026-05-18   Declination: 3° 28' 56" E
        //              changing by +0° 7' 24"/yr
        //              uncertainty ±0° 20' (1σ)
        const DECLINATION_DEG: f32 = 3.0 + 28.0 / 60.0 + 56.0 / 3600.0;

        const DECLINATION_RAD: f32 = DECLINATION_DEG.to_radians();

        // Fusion aligns its +x with the measured horizontal field (magnetic north).
        // Mag-NED is the true-NED frame rotated by +declination about z-down, so
        // body -> true-NED is R_z(+declination) * body -> mag-NED.
        let orientation_true =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), DECLINATION_RAD) * orientation_mag;

        let orientation = orientation_true;

        const STANDARD_G: f32 = 9.80665; // m/s^2
        const GRAVITY: Vector3<f32> = Vector3::new(0.0, 0.0, STANDARD_G);

        // now we can work on the inertial data.
        let body_accel_raw = Vector3::new(accel.x, accel.y, accel.z); // g
        let body_angular_velocity = Vector3::new(gyro.x, gyro.y, gyro.z); // deg/s

        // we need to convert accel to m/s^2, angular velocity stays in deg/s
        let body_accel_scaled = body_accel_raw * STANDARD_G;

        // we need to rotate from body into inertial frame. the quaternion is a passive rotation.
        let inertial_accel = orientation.transform_vector(&body_accel_scaled);
        let inertial_gyro = orientation.transform_vector(&body_angular_velocity);

        // now we have to compensate for gravity
        let inertial_accel_compensated = inertial_accel + GRAVITY;

        let inertial_data = ImuData {
            acceleration_x: body_accel_scaled.x,
            acceleration_y: body_accel_scaled.y,
            acceleration_z: body_accel_scaled.z,
            angular_velocity_x: body_angular_velocity.x,
            angular_velocity_y: body_angular_velocity.y,
            angular_velocity_z: body_angular_velocity.z,
            acceleration_north: inertial_accel_compensated.x,
            acceleration_east: inertial_accel_compensated.y,
            acceleration_down: inertial_accel_compensated.z,
            angular_velocity_north: inertial_gyro.x,
            angular_velocity_east: inertial_gyro.y,
            angular_velocity_down: inertial_gyro.z,
        };

        if realtime {
            // Only publish the latest orientation to watch
            self.orientation_watch.send(orientation);
            self.inertial_watch.send(inertial_data.clone());
        }
        self.orientation_pub.publish_immediate(orientation);
        self.inertial_pub.publish_immediate(inertial_data);
    }
}

struct TimeoutSelector {
    primary: SensorId,
    last_update: Instant,
    timeout: Duration,
}

impl TimeoutSelector {
    fn new(timeout: Duration) -> Self {
        Self {
            primary: SensorId::Imu1,
            last_update: Instant::now(),
            timeout,
        }
    }

    fn accept(&mut self, id: SensorId, ts: Instant) -> bool {
        if self.primary == id || self.last_update.elapsed() > self.timeout {
            self.primary = id;
            self.last_update = ts;
            true
        } else {
            false
        }
    }
}

fn fusion_instance() -> Fusion {
    let mut s = FusionAhrsSettings::new();
    s.gyr_range = 2_000.0;
    s.gain = 2.0;
    s.acc_rejection = 10.0;
    s.recovery_trigger_period = 300; //~8s
    s.mag_rejection = 10.0;
    s.convention = imu_fusion::FusionConvention::NED;
    Fusion::new(IMU_ODR_HZ, s)
}
