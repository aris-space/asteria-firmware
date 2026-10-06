//! Prints the estimated height and upward velocity over defmt at 20 Hz.

use defmt::info;
use embassy_time::{Duration, Ticker};

use crate::signals;

const PERIOD: Duration = Duration::from_millis(50);

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut rx = signals::STATE_ESTIMATE_WATCH
        .receiver()
        .expect("state report: estimate receiver available");
    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;
        if let Some(estimate) = rx.try_changed() {
            info!(
                "height {=f32} ± {=f32} m, velocity up {=f32} ± {=f32} m/s",
                -estimate.position_ned_m[2],
                estimate.position_std_ned_m[2],
                -estimate.velocity_ned_mps[2],
                estimate.velocity_std_ned_mps[2]
            );
        }
    }
}
