use core::sync::atomic::Ordering;

use defmt::{Format, debug, warn};
use embassy_time::{Duration, Instant, Timer};

use crate::sensors::{AtomicSensorStatus, SensorStatus};

pub mod baro;
pub mod dht;
pub mod gnss;
pub mod imu;
pub mod mag;

/// Consecutive read errors before a readout returns to `Inactive`.
pub const MAX_CONSECUTIVE_ERRORS: u8 = 10;

/// Initialization attempts an I2C readout makes at startup.
pub const MAX_INIT_ATTEMPTS: u8 = 3;

// needed to limit startup delays from missing I2C sensors
const BASE_BACKOFF_MS: u64 = 100;
const MAX_BACKOFF_MS: u64 = 5000;

pub fn backoff(attempt: u8) -> Duration {
    let ms = (BASE_BACKOFF_MS << attempt.min(6)).min(MAX_BACKOFF_MS);
    Duration::from_millis(ms)
}

/// One state of a readout's `Inactive` <-> `Active` cycle.
trait State {
    type Next;
    async fn run(self) -> Self::Next;
}

/// Runs a readout forever, starting from `inactive`.
async fn run<I, A>(status: &AtomicSensorStatus, mut inactive: I) -> !
where
    I: State<Next = A>,
    A: State<Next = I>,
{
    loop {
        let active = inactive.run().await;
        status.store(SensorStatus::Active, Ordering::Relaxed);
        inactive = active.run().await;
        status.store(SensorStatus::Inactive, Ordering::Relaxed);
    }
}

/// Startup initialization of an I2C sensor, bounded by [`MAX_INIT_ATTEMPTS`].
async fn init_at_startup<S>(
    id: impl Format,
    mut configure: impl AsyncFnMut() -> Result<S, ()>,
) -> Option<S> {
    for attempt in 1..=MAX_INIT_ATTEMPTS {
        debug!("{}: initializing (attempt {})", id, attempt);
        if let Ok(sensor) = configure().await {
            debug!("{}: initialized", id);
            return Some(sensor);
        }
        if attempt < MAX_INIT_ATTEMPTS {
            Timer::after(backoff(attempt)).await;
        }
    }
    warn!("{}: not detected; retrying in the background", id);
    None
}

/// Waits for the next sample of a fixed-rate readout.
async fn wait_for_sample(next_sample: Instant, id: impl Format) {
    if Instant::now() > next_sample {
        warn!("{}: can't keep up with sample interval", id);
    } else {
        Timer::at(next_sample).await;
    }
}
