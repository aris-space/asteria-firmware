//! IMU-driven EKF prediction with GNSS position correction.
//!
//! Subscribes to:
//!   - `INERTIAL_CHANNELS[IMU_0]` (calibrated inertial samples) — runs the
//!     `ekf` crate's `Imu` predictor with `f64`.
//!   - `GNSS_CHANNELS[GNSS_0]`   — applies a 3-D position correction via
//!     `ekf::measurements::Gnss::from_geodetic` once the NED origin has been
//!     anchored on the first valid 3D fix.
//!
//! Publishes the latest state snapshot to `signals::STATE_ESTIMATE_WATCH`
//! after each predict step (correction-only updates also publish).

use defmt::{debug, info, trace, warn};
use ekf::measurements::gnss::{GeodeticOrigin, Gnss};
use ekf::nalgebra::{UnitQuaternion, Vector3};
use ekf::predictors::imu::{AccelConvention, Imu, ImuNoise};
use ekf::{
    CovMatrix, Ekf, IDX_AX, IDX_AY, IDX_AZ, IDX_D, IDX_E, IDX_N, IDX_VD, IDX_VE, IDX_VN, STATE_DIM,
    StateVec,
};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant};
use static_cell::StaticCell;
use ublox::GpsFix;

use crate::measurements::StateEstimate;
#[cfg(feature = "profiling")]
use crate::profiling;
use crate::sensors::{GNSS_0, GnssId, IMU_0, ImuId};
use crate::signals;

/// Convert accelerometer reading (g) to m/s².
const G_TO_MPS2: f64 = 9.81;
/// Convert gyroscope reading (deg/s) to rad/s.
const DPS_TO_RADPS: f64 = core::f64::consts::PI / 180.0;

// TODO: move noise parameters into `params/` once a NVM-backed config slot
// exists for them. Numbers are first-cut estimates for the LSM6DSO32 at
// 833 Hz / ±8 g / ±2000 dps, derived from datasheet noise densities.
const ACCEL_STDDEV_MPS2: f64 = 0.05;
const GYRO_STDDEV_RADPS: f64 = 0.005;

/// Which IMU drives the EKF. TODO: support failover to IMU_1 if IMU_0 stops
/// publishing, or fuse both streams once we have a multi-IMU prediction model.
const PRIMARY_IMU: ImuId = IMU_0;
/// Which GNSS receiver corrects the EKF. TODO: failover to GNSS_1 / dual-receiver fusion.
const PRIMARY_GNSS: GnssId = GNSS_0;
/// Reject GNSS fixes below this satellite count. Tuneable; 5 keeps us above
/// the bare 4-satellite minimum for a 3D fix and rejects glitches at startup.
const MIN_SATELLITES_FOR_FIX: u8 = 5;

static EKF_CELL: StaticCell<Ekf<f64>> = StaticCell::new();

fn vec3_from_xyz_f32(x: f32, y: f32, z: f32, scale: f64) -> Vector3<f64> {
    Vector3::new(x as f64 * scale, y as f64 * scale, z as f64 * scale)
}

fn snapshot(ekf: &Ekf<f64>, ts: Instant) -> StateEstimate {
    let mut cov_diag = [0.0_f64; STATE_DIM];
    for i in 0..STATE_DIM {
        cov_diag[i] = ekf.covariance[(i, i)];
    }
    let q = ekf.attitude.into_inner();
    StateEstimate {
        ts,
        pos_ned_m: [ekf.state[IDX_N], ekf.state[IDX_E], ekf.state[IDX_D]],
        vel_ned_mps: [ekf.state[IDX_VN], ekf.state[IDX_VE], ekf.state[IDX_VD]],
        // nalgebra's quaternion stores [i, j, k, w]; reorder to [w, x, y, z].
        attitude_quat_wxyz: [q.w, q.i, q.j, q.k],
        cov_diag,
    }
}

#[embassy_executor::task]
pub async fn task() -> ! {
    let ekf: &mut Ekf<f64> = EKF_CELL.init(Ekf {
        state: StateVec::zeros(),
        covariance: CovMatrix::zeros(),
        attitude: UnitQuaternion::identity(),
    });
    ekf.init();

    let mut imu = Imu::<f64>::default()
        .set_accel_convention(AccelConvention::SpecificForce)
        .set_noise(ImuNoise::new(ACCEL_STDDEV_MPS2, GYRO_STDDEV_RADPS));
    // TODO: do a static accel-based attitude alignment on the first N
    // stationary samples instead of starting from identity. Without
    // magnetometer corrections, attitude is unobservable around vertical.

    let publisher = signals::STATE_ESTIMATE_WATCH.sender();
    let mut imu_sub = signals::INERTIAL_CHANNELS[PRIMARY_IMU.index()]
        .subscriber()
        .expect("INERTIAL_CHANNELS subscriber slot");
    let mut gnss_sub = signals::GNSS_CHANNELS[PRIMARY_GNSS.index()]
        .subscriber()
        .expect("GNSS_CHANNELS subscriber slot");

    let mut last_ts: Option<Instant> = None;
    // NED frame anchor — set on the first valid 3D fix. TODO: persist via
    // params/ once that subsystem is reliable so we don't re-anchor on every
    // boot.
    let mut origin: Option<GeodeticOrigin<f64>> = None;
    let mut next_pos_log_at: Instant = Instant::from_ticks(0);

    #[cfg(feature = "profiling")]
    let predict_stats = profiling::CycleStats::new();
    #[cfg(feature = "profiling")]
    let correct_stats = profiling::CycleStats::new();
    /// Log timing + stack stats every N predicts (~833 Hz → ~1.2 s at 1024).
    #[cfg(feature = "profiling")]
    const LOG_EVERY: u32 = 1024;
    #[cfg(feature = "profiling")]
    let mut step_counter: u32 = 0;

    loop {
        match select(imu_sub.next_message_pure(), gnss_sub.next_message_pure()).await {
            Either::First(sample) => {
                let ts = sample.data.ts;
                let Some(prev) = last_ts.replace(ts) else {
                    // First sample: only seed last_ts, no dt to predict against yet.
                    continue;
                };

                let dt_us = ts.checked_duration_since(prev).map(|d| d.as_micros());
                let Some(dt_us) = dt_us else {
                    // Out-of-order timestamp; skip but keep last_ts updated to ts.
                    warn!("ekf: non-monotonic IMU timestamp, skipping step");
                    continue;
                };
                // TODO: clamp absurd dt (sensor stall / startup glitch) once we
                // have a defined recovery policy. For now trust the readout.
                let dt = dt_us as f64 * 1e-6;

                let accel = vec3_from_xyz_f32(
                    sample.data.value.accel.x,
                    sample.data.value.accel.y,
                    sample.data.value.accel.z,
                    G_TO_MPS2,
                );
                let gyro = vec3_from_xyz_f32(
                    sample.data.value.gyro.x,
                    sample.data.value.gyro.y,
                    sample.data.value.gyro.z,
                    DPS_TO_RADPS,
                );
                imu.update_measurement(
                    accel,
                    gyro,
                    ImuNoise::new(ACCEL_STDDEV_MPS2, GYRO_STDDEV_RADPS),
                );

                #[cfg(feature = "profiling")]
                let t0 = profiling::cycle_count();

                ekf.predict(&mut imu, dt);

                #[cfg(feature = "profiling")]
                predict_stats.observe(profiling::cycle_count().wrapping_sub(t0));

                publisher.send(snapshot(ekf, ts));
                trace!("ekf: predicted, dt={} us", dt_us);

                if ts >= next_pos_log_at {
                    info!(
                        "ekf pos NED (m): n={} e={} d={}",
                        ekf.state[IDX_N], ekf.state[IDX_E], ekf.state[IDX_D],
                    );
                    next_pos_log_at = ts + Duration::from_secs(1);
                }

                #[cfg(feature = "profiling")]
                {
                    step_counter = step_counter.wrapping_add(1);
                    if step_counter % LOG_EVERY == 0 {
                        let (p_min, p_avg, p_max, p_n) = predict_stats.snapshot_and_reset();
                        let (c_min, c_avg, c_max, c_n) = correct_stats.snapshot_and_reset();
                        let hw = profiling::msp_high_water().unwrap_or(0);
                        info!(
                            "ekf profile: predict cycles min={} avg={} max={} (n={}, ~{} us avg, ~{} us max); correct cycles min={} avg={} max={} (n={}, ~{} us avg, ~{} us max); MSP high-water {} bytes",
                            p_min,
                            p_avg,
                            p_max,
                            p_n,
                            profiling::cycles_to_us(p_avg),
                            profiling::cycles_to_us(p_max),
                            c_min,
                            c_avg,
                            c_max,
                            c_n,
                            profiling::cycles_to_us(c_avg),
                            profiling::cycles_to_us(c_max),
                            hw,
                        );
                    }
                }
            }

            Either::Second(sample) => {
                let pvt = &sample.data.value;

                // Quality gate: only fold in 3D fixes with enough satellites.
                if !matches!(pvt.fix_type, GpsFix::Fix3D) {
                    continue;
                }
                if pvt.num_satellites < MIN_SATELLITES_FOR_FIX {
                    continue;
                }

                let alt = pvt.height_ellipsoid_m as f64;
                let lat = pvt.lat_deg;
                let lon = pvt.lon_deg;

                // Anchor the NED frame on the first qualifying fix.
                let origin_ref = origin.get_or_insert_with(|| {
                    debug!(
                        "ekf: anchoring NED origin at lat={}, lon={}, alt_ellipsoid={}",
                        lat, lon, alt,
                    );
                    GeodeticOrigin {
                        lat_deg: lat,
                        lon_deg: lon,
                        alt_m: alt,
                    }
                });

                // ublox accuracies are 1-sigma in millimetres. Convert to metres.
                let h_acc_m = pvt.horiz_accuracy as f64 * 1e-3;
                let v_acc_m = pvt.vert_accuracy as f64 * 1e-3;
                let stddev = Vector3::new(h_acc_m, h_acc_m, v_acc_m);

                let mut measurement = Gnss::<f64>::from_geodetic(lat, lon, alt, origin_ref, stddev);

                // TODO: out-of-order handling. The GNSS readout backdates each
                // sample by GNSS_DELAY (~100 ms) but the EKF state represents
                // "now". For milestone 1 we fold the correction into the
                // current state and accept the resulting position bias
                // (~v*0.1, e.g. 5 m at 50 m/s).
                #[cfg(feature = "profiling")]
                let t0 = profiling::cycle_count();

                ekf.correct(&mut measurement);

                #[cfg(feature = "profiling")]
                correct_stats.observe(profiling::cycle_count().wrapping_sub(t0));

                publisher.send(snapshot(ekf, sample.data.ts));
                trace!(
                    "ekf: corrected from GNSS, sats={}, h_acc={} mm, v_acc={} mm",
                    pvt.num_satellites, pvt.horiz_accuracy, pvt.vert_accuracy,
                );
            }
        }
    }
}

// Compile-time guard: keep an eye on the static EKF size in case we add
// f64 fields. 9-state f64 EKF = 9*8 (state) + 81*8 (cov) + 4*8 (quat) = 728 B.
// Picking 1 KiB as a soft limit; bump deliberately if it ever needs to grow.
const _: () = assert!(core::mem::size_of::<Ekf<f64>>() < 1024);

// Indices imported but not yet used here — they'll be needed once correction
// measurements are wired in.
#[allow(dead_code)]
const _UNUSED_INDICES: [usize; 3] = [IDX_AX, IDX_AY, IDX_AZ];
