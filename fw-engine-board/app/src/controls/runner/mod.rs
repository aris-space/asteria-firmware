use crate::controls::actions::watch::watch_task_runner;
use crate::controls::runner::abort::abort_task_runner;
use crate::controls::runner::firing::firing_task_runner;
use crate::drivers::{CAP, PUB, SUB, WATCH};
use embassy_executor::{SpawnError, Spawner};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::Watch;
use embedded_utils::info;

mod abort;
mod firing;

// Channels to initiate a firing or abort
pub static FIRING_INITIATION: Watch<CriticalSectionRawMutex, (), WATCH> = Watch::new();
pub static ABORT_INITIATION: Watch<CriticalSectionRawMutex, (), WATCH> = Watch::new();
// Channels to notify firing or abort completion
pub static FIRING_INFO: PubSubChannel<CriticalSectionRawMutex, FiringInfo, CAP, SUB, PUB> =
    PubSubChannel::new();

#[derive(Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FiringInfo {
    FiringInitiated,
    CombustionDetected,
    FiringCompleted,
    FiringAborted,
}

pub async fn initiate_runner_tasks(spawner: Spawner) -> Result<(), SpawnError> {
    spawner.spawn(firing_task_runner()?);
    spawner.spawn(abort_task_runner()?);
    spawner.spawn(watch_task_runner()?);

    info!("Firing Task Runners Initiated");
    Ok(())
}
