//! Prints every estimated height and upward velocity over defmt.

use defmt::info;

use crate::signals;

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut rx = signals::STATE_ESTIMATE_WATCH
        .receiver()
        .expect("state report: estimate receiver available");
    loop {
        let estimate = rx.changed().await;
        info!(
            "height {=f32} m, velocity up {=f32} m/s",
            estimate.height_msl_m, estimate.velocity_mps
        );
    }
}
