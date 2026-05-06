use crate::controls::actions::ActionCompleteness;
use crate::drivers::WATCH;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Receiver;
use embassy_time::{Duration, Instant, Timer};

const WAIT_POLL_INTERVAL: Duration = Duration::from_millis(10);

pub async fn wait_with_abort<'a>(
    duration: &Duration,
    abort_initiation_receiver: &mut Receiver<'a, CriticalSectionRawMutex, (), WATCH>,
) -> ActionCompleteness {
    let start = Instant::now();
    loop {
        if abort_initiation_receiver.try_changed().is_some() {
            return ActionCompleteness::Failed;
        }

        if start.elapsed() > *duration {
            return ActionCompleteness::Successful;
        }
        Timer::after(WAIT_POLL_INTERVAL).await;
    }
}

pub async fn wait_no_abort(duration: &Duration) {
    let start = Instant::now();
    while start.elapsed() < *duration {
        Timer::after(WAIT_POLL_INTERVAL).await;
    }
}
