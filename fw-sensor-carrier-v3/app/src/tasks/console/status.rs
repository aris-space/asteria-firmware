//! `status`: five seconds of every sensor stream and the estimator, with the
//! applied calibration of each sensor.

use core::sync::atomic::Ordering;

use embassy_futures::select::{Either6, select6};
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Instant, Timer};
use nalgebra::Vector3;

use super::io::{ConsoleIo, say, sayf};
use crate::calibration::{Calibrations, Correction, StoredCal, baro, gnss, imu, mag};
use crate::sensors::{
    AtomicSensorStatus, BARO_COUNT, BARO_STATUS, GNSS_COUNT, GNSS_STATUS, IMU_COUNT, IMU_STATUS,
    MAG_COUNT, MAG_STATUS, SensorId, SensorStatus,
};
use crate::signals;
use crate::tasks::state_estimation::{MAX_MAG_FIELD_ERROR, mag_field_error};
use crate::types::{BaroSample, GnssSample, ImuSample, MagSample, Pvt, SefLogSample};

const WINDOW: Duration = Duration::from_secs(5);

#[derive(Clone, Copy, Default)]
struct ImuStats {
    count: u32,
    accel_norm_g: f32,
    gyro_dps: [f32; 3],
}

impl ImuStats {
    fn add(&mut self, sample: &ImuSample) {
        let accel = Vector3::new(sample.accel.x, sample.accel.y, sample.accel.z);
        self.count += 1;
        self.accel_norm_g += accel.norm();
        for (sum, rate) in
            self.gyro_dps
                .iter_mut()
                .zip([sample.gyro.x, sample.gyro.y, sample.gyro.z])
        {
            *sum += rate;
        }
    }
}

#[derive(Clone, Copy, Default)]
struct MagStats {
    count: u32,
    fused: u32,
    field_nt: [f32; 3],
}

impl MagStats {
    fn add(&mut self, sample: &MagSample) {
        let field_nt = [sample.x, sample.y, sample.z];
        self.count += 1;
        if libm::fabsf(mag_field_error(field_nt)) <= MAX_MAG_FIELD_ERROR {
            self.fused += 1;
        }
        for (sum, component) in self.field_nt.iter_mut().zip(field_nt) {
            *sum += component;
        }
    }

    fn mean_nt(&self) -> Option<[f32; 3]> {
        (self.count > 0).then(|| self.field_nt.map(|sum| sum / self.count as f32))
    }
}

#[derive(Clone, Copy, Default)]
struct BaroStats {
    count: u32,
    pressure_mbar: f32,
    temperature_c: f32,
}

impl BaroStats {
    fn add(&mut self, sample: &BaroSample) {
        self.count += 1;
        self.pressure_mbar += sample.pressure_mbar;
        self.temperature_c += sample.temperature_c;
    }
}

#[derive(Clone, Copy, Default)]
struct GnssStats {
    count: u32,
    last: Option<Pvt>,
}

impl GnssStats {
    fn add(&mut self, sample: &GnssSample) {
        self.count += 1;
        self.last = Some(sample.pvt);
    }
}

#[derive(Clone, Copy, Default)]
struct ChainStats {
    count: u32,
    magnetometer_ignored: u32,
    last: Option<SefLogSample>,
}

impl ChainStats {
    fn add(&mut self, sample: &SefLogSample) {
        self.count += 1;
        self.magnetometer_ignored += u32::from(sample.magnetometer_ignored);
        self.last = Some(*sample);
    }
}

#[derive(Default)]
struct Window {
    imu: [ImuStats; IMU_COUNT],
    mag: [MagStats; MAG_COUNT],
    baro: [BaroStats; BARO_COUNT],
    gnss: [GnssStats; GNSS_COUNT],
    chains: [ChainStats; IMU_COUNT],
    lost: u64,
}

impl Window {
    async fn record() -> Result<Self, &'static str> {
        let mut imu = signals::IMU_CHANNEL.subscriber().map_err(|_| "imu")?;
        let mut mag = signals::MAG_CHANNEL.subscriber().map_err(|_| "mag")?;
        let mut baro = signals::BARO_CHANNEL.subscriber().map_err(|_| "baro")?;
        let mut gnss = signals::GNSS_CHANNEL.subscriber().map_err(|_| "gnss")?;
        let mut chains = signals::STATE_CHANNEL.subscriber().map_err(|_| "state")?;
        let mut window = Self::default();
        let end = Instant::now() + WINDOW;
        loop {
            match select6(
                imu.next_message(),
                mag.next_message(),
                baro.next_message(),
                gnss.next_message(),
                chains.next_message(),
                Timer::at(end),
            )
            .await
            {
                Either6::First(WaitResult::Message(r)) => window.imu[r.cal.src.index()].add(&r.cal),
                Either6::Second(WaitResult::Message(r)) => {
                    window.mag[r.cal.src.index()].add(&r.cal)
                }
                Either6::Third(WaitResult::Message(r)) => {
                    window.baro[r.cal.src.index()].add(&r.cal)
                }
                Either6::Fourth(WaitResult::Message(r)) => {
                    window.gnss[r.cal.src.index()].add(&r.cal)
                }
                Either6::Fifth(WaitResult::Message(s)) => window.chains[s.imu.index()].add(&s),
                Either6::First(WaitResult::Lagged(lost))
                | Either6::Second(WaitResult::Lagged(lost))
                | Either6::Third(WaitResult::Lagged(lost))
                | Either6::Fourth(WaitResult::Lagged(lost))
                | Either6::Fifth(WaitResult::Lagged(lost)) => window.lost += lost,
                Either6::Sixth(()) => return Ok(window),
            }
        }
    }
}

pub async fn command(io: &mut ConsoleIo<'_>) {
    sayf(io, format_args!("sampling for {} s\n", WINDOW.as_secs())).await;
    let window = match Window::record().await {
        Ok(window) => window,
        Err(channel) => {
            sayf(
                io,
                format_args!(paint!(red, "status: no free {} subscriber\n"), channel),
            )
            .await;
            return;
        }
    };

    say(io, "estimator\n").await;
    for (id, chain) in imu::CAL.ids().into_iter().zip(window.chains) {
        let Some(last) = chain.last else {
            sayf(
                io,
                format_args!("  {} {}\n", id.name(), paint!(red, "no estimate")),
            )
            .await;
            continue;
        };
        let magnetometer = if chain.magnetometer_ignored == 0 {
            paint!(green, "magnetometer used")
        } else {
            paint!(red, "magnetometer used")
        };
        sayf(
            io,
            format_args!(
                "  {} {} height {:.2} ± {:.2} m, velocity {:.3} ± {:.3} m/s, score {:.2}, {} {}/{}\n",
                id.name(),
                if last.selected { "selected" } else { "standby" },
                last.height_msl_m,
                last.height_std_m,
                last.velocity_mps,
                last.velocity_std_mps,
                last.consistency_score,
                magnetometer,
                chain.count - chain.magnetometer_ignored,
                chain.count,
            ),
        )
        .await;
    }
    if let Some(last) = window.chains.iter().find_map(|chain| chain.last) {
        sayf(
            io,
            format_args!(
                "  msl height {}, redundancy {}\n",
                ready(last.msl_ready),
                ready(last.redundancy_ready)
            ),
        )
        .await;
    }

    say(io, "imu\n").await;
    for (id, stats) in imu::CAL.ids().into_iter().zip(window.imu) {
        sensor(io, id, &IMU_STATUS, &imu::CAL, stats.count).await;
        if stats.count > 0 {
            let n = stats.count as f32;
            let [x, y, z] = stats.gyro_dps.map(|sum| sum / n);
            sayf(
                io,
                format_args!(
                    "    |a| {:.4} g, gyro {:.3}, {:.3}, {:.3} dps\n",
                    stats.accel_norm_g / n,
                    x,
                    y,
                    z
                ),
            )
            .await;
        }
    }

    say(io, "magnetometer\n").await;
    for (id, stats) in mag::CAL.ids().into_iter().zip(window.mag) {
        sensor(io, id, &MAG_STATUS, &mag::CAL, stats.count).await;
        if let Some(field_nt) = stats.mean_nt() {
            let fused = if stats.fused == stats.count {
                paint!(green, "fused")
            } else {
                paint!(red, "fused")
            };
            sayf(
                io,
                format_args!(
                    "    |B| {:.1} uT, {:+.1} % off the local field, {} {}/{}\n",
                    Vector3::from(field_nt).norm() / 1000.0,
                    mag_field_error(field_nt) * 100.0,
                    fused,
                    stats.fused,
                    stats.count
                ),
            )
            .await;
        }
    }
    if let [Some(a), Some(b)] = window.mag.map(|stats| stats.mean_nt()) {
        let difference_nt = (Vector3::from(a) - Vector3::from(b)).norm();
        sayf(
            io,
            format_args!("  difference {:.1} uT\n", difference_nt / 1000.0),
        )
        .await;
    }

    say(io, "barometer\n").await;
    for (id, stats) in baro::CAL.ids().into_iter().zip(window.baro) {
        sensor(io, id, &BARO_STATUS, &baro::CAL, stats.count).await;
        if stats.count > 0 {
            let n = stats.count as f32;
            sayf(
                io,
                format_args!(
                    "    {:.2} mbar, {:.1} C\n",
                    stats.pressure_mbar / n,
                    stats.temperature_c / n
                ),
            )
            .await;
        }
    }

    say(io, "gnss\n").await;
    for (id, stats) in gnss::CAL.ids().into_iter().zip(window.gnss) {
        sensor(io, id, &GNSS_STATUS, &gnss::CAL, stats.count).await;
        if let Some(pvt) = stats.last {
            let fix = if pvt.has_3d_fix() {
                paint!(green, "3D fix")
            } else {
                paint!(red, "no 3D fix")
            };
            sayf(
                io,
                format_args!(
                    "    {}, {}, {} satellites, accuracy {:.2} m horizontal, {:.2} m vertical, {:.2} m/s, pdop {:.2}\n",
                    if signals::SELECTED_GNSS.try_get() == Some(id) {
                        "selected"
                    } else {
                        "standby"
                    },
                    fix,
                    pvt.num_satellites,
                    pvt.horizontal_accuracy_mm as f32 / 1000.0,
                    pvt.vertical_accuracy_mm as f32 / 1000.0,
                    pvt.speed_accuracy_mps,
                    f32::from(pvt.pdop_centi) / 100.0
                ),
            )
            .await;
        }
    }

    match (signals::SD_LOGGING.try_get(), signals::SD_SESSION.try_get()) {
        (Some(true), Some(session)) => {
            sayf(
                io,
                format_args!(
                    "sd card {} to {}, {} readings dropped\n",
                    paint!(green, "logging"),
                    session.log,
                    session.dropped
                ),
            )
            .await
        }
        _ => sayf(io, format_args!("sd card {}\n", paint!(red, "not logging"))).await,
    }
    if window.lost > 0 {
        sayf(
            io,
            format_args!("status fell behind and missed {} readings\n", window.lost),
        )
        .await;
    }
}

async fn sensor<Id: SensorId, C: Correction, const N: usize>(
    io: &mut ConsoleIo<'_>,
    id: Id,
    statuses: &[AtomicSensorStatus],
    cals: &Calibrations<Id, C, N>,
    count: u32,
) {
    let status = match statuses[id.index()].load(Ordering::Relaxed) {
        SensorStatus::Active => paint!(green, "active"),
        SensorStatus::Inactive => paint!(red, "inactive"),
    };
    let cal = cals.applied(id);
    let rate_hz = count as f32 / WINDOW.as_secs() as f32;
    if cal == StoredCal::DEFAULT {
        sayf(
            io,
            format_args!(
                "  {} {}, {}, {:.0} Hz\n",
                id.name(),
                status,
                paint!(red, "not calibrated"),
                rate_hz
            ),
        )
        .await;
    } else {
        sayf(
            io,
            format_args!(
                "  {} {}, cal {}, {:.0} Hz\n",
                id.name(),
                status,
                cal.name,
                rate_hz
            ),
        )
        .await;
    }
}

fn ready(ready: bool) -> &'static str {
    if ready {
        paint!(green, "ready")
    } else {
        paint!(red, "not ready")
    }
}
