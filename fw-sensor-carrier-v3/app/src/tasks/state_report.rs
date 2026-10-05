//! Prints the latest state estimate over defmt once per second.

use defmt::info;
use embassy_time::{Duration, Ticker};
use nalgebra::{Quaternion, UnitQuaternion};

use crate::signals;

const PERIOD: Duration = Duration::from_secs(1);

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut rx = signals::STATE_ESTIMATE_WATCH
        .receiver()
        .expect("state report: estimate receiver available");
    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;
        let Some(estimate) = rx.try_changed() else {
            info!("SEF: no new estimate");
            continue;
        };
        let [w, x, y, z] = estimate.orientation_body_to_ned_wxyz;
        let (roll, pitch, yaw) =
            UnitQuaternion::from_quaternion(Quaternion::new(w, x, y, z)).euler_angles();
        info!(
            "SEF: {} height {=f32} ± {=f32} m ({}), velocity up {=f32} ± {=f32} m/s, roll {=f32} pitch {=f32} yaw {=f32} deg, redundancy {}",
            estimate.selected_imu,
            estimate.height_msl_m,
            estimate.height_std_m,
            if estimate.msl_ready {
                "MSL"
            } else {
                "not yet MSL"
            },
            estimate.velocity_mps,
            estimate.velocity_std_mps,
            roll.to_degrees(),
            pitch.to_degrees(),
            yaw.to_degrees(),
            if estimate.redundancy_ready {
                "ready"
            } else {
                "not ready"
            },
        );
    }
}
