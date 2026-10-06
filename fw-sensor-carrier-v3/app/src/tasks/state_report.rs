//! Prints the estimated height, upward velocity, and barometer biases over defmt at 20 Hz.

use defmt::info;
use embassy_sync::pubsub::WaitResult;
use embassy_time::{Duration, Ticker};

use crate::signals;

const PERIOD: Duration = Duration::from_millis(50);

#[embassy_executor::task]
pub async fn task() -> ! {
    let mut rx = signals::STATE_ESTIMATE_WATCH
        .receiver()
        .expect("state report: estimate receiver available");
    // The biases are SEF-light's own, so they come from its per-chain log samples.
    let mut chains = signals::STATE_CHANNEL
        .subscriber()
        .expect("state report: chain subscriber available");
    let mut selected = None;
    let mut ticker = Ticker::every(PERIOD);
    loop {
        ticker.next().await;
        while let Some(message) = chains.try_next_message() {
            if let WaitResult::Message(chain) = message
                && chain.selected
            {
                selected = Some(chain);
            }
        }
        if let Some(estimate) = rx.try_changed() {
            let (bias_m, bias_std_m) = selected.map_or(([f32::NAN; 2], [f32::NAN; 2]), |chain| {
                (chain.barometer_bias_m, chain.barometer_bias_std_m)
            });
            info!(
                "height {=f32} ± {=f32} m, velocity up {=f32} ± {=f32} m/s, baro bias {=f32} ± {=f32} / {=f32} ± {=f32} m",
                -estimate.position_ned_m[2],
                estimate.position_std_ned_m[2],
                -estimate.velocity_ned_mps[2],
                estimate.velocity_std_ned_mps[2],
                bias_m[0],
                bias_std_m[0],
                bias_m[1],
                bias_std_m[1]
            );
            /*
            info!(
                "orientation body-to-NED q [w, x, y, z]: {=f32}, {=f32}, {=f32}, {=f32}",
                estimate.orientation_body_to_ned_wxyz[0],
                estimate.orientation_body_to_ned_wxyz[1],
                estimate.orientation_body_to_ned_wxyz[2],
                estimate.orientation_body_to_ned_wxyz[3],
            );
            info!(
                "orientation std N/E/D [rad]: {=f32}, {=f32}, {=f32}",
                estimate.orientation_std_ned_rad[0],
                estimate.orientation_std_ned_rad[1],
                estimate.orientation_std_ned_rad[2],
            );
            info!(
                "angular rate body X/Y/Z [rad/s]: X {=f32} ± {=f32}, Y {=f32} ± {=f32}, Z {=f32} ± {=f32}",
                estimate.angular_rate_body_rad_s[0], estimate.angular_rate_std_body_rad_s[0],
                estimate.angular_rate_body_rad_s[1], estimate.angular_rate_std_body_rad_s[1],
                estimate.angular_rate_body_rad_s[2], estimate.angular_rate_std_body_rad_s[2],
            );
            info!(
                "specific force body X/Y/Z [m/s^2]: X {=f32} ± {=f32}, Y {=f32} ± {=f32}, Z {=f32} ± {=f32}",
                estimate.specific_force_body_mps2[0], estimate.specific_force_std_body_mps2[0],
                estimate.specific_force_body_mps2[1], estimate.specific_force_std_body_mps2[1],
                estimate.specific_force_body_mps2[2], estimate.specific_force_std_body_mps2[2],
            );
            */
        }
    }
}
