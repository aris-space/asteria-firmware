use crate::buzzer::{BuzzerState, BUZZER_WATCH};
use crate::drivers::{CAP, PUB, SUB, WATCH};
use core::future::pending;
use embassy_futures::join::join;
use embassy_stm32::gpio::{Input, Level, Output};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::watch::{Receiver, Watch};
use embassy_time::{Duration, Timer};
use hermes_can::messages::board_status::{ArmingState, ValveState};
use hermes_can::messages::event_messages::DprState;

pub static OSS_MAIN_CONTROL: Watch<ThreadModeRawMutex, ValveState, WATCH> = Watch::new();
pub static FSS_MAIN_CONTROL: Watch<ThreadModeRawMutex, ValveState, WATCH> = Watch::new();
pub static EXTERNAL_VALVE_CONTROL: PubSubChannel<ThreadModeRawMutex, ExternalValve, CAP, SUB, PUB> =
    PubSubChannel::new();
pub static MAIN_ARMING: Mutex<ThreadModeRawMutex, ArmingState> = Mutex::new(ArmingState::Safe);

pub const MAIN_ARMING_UPDATE_RATE: Duration = Duration::from_millis(1000);

#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OnboardValve {
    FuelMain(ValveState),
    OxidizerMain(ValveState),
}

#[allow(dead_code)]
#[derive(Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ExternalValve {
    NitrogenVent(ValveState),
    FuelDpr(DprState),
    FuelVent(ValveState),
    OxidizerDpr(DprState),
    OxidizerVent(ValveState),
    IgniterFuel(ValveState),
    IgniterOxidizer(ValveState),
    IgniterPurge(ValveState),
    IgniterSpark(ValveState),
}

#[embassy_executor::task]
pub(crate) async fn valve_task(oss_mnl_vlv: Output<'static>, fss_mnl_vlv: Output<'static>) {
    let oss_mnl_vlv_watcher = OSS_MAIN_CONTROL.receiver().unwrap();
    let fss_mnl_vlv_watcher = FSS_MAIN_CONTROL.receiver().unwrap();
    let oss_mnl_vlv_task = valve_task_impl(oss_mnl_vlv, oss_mnl_vlv_watcher);
    let fss_mnl_vlv_task = valve_task_impl(fss_mnl_vlv, fss_mnl_vlv_watcher);

    join(oss_mnl_vlv_task, fss_mnl_vlv_task).await;

    loop {
        pending::<()>().await;
    }
}

async fn valve_task_impl(
    mut valve: Output<'static>,
    mut watch: Receiver<'static, ThreadModeRawMutex, ValveState, WATCH>,
) {
    loop {
        let state = watch.changed().await;
        match state {
            ValveState::Active => {
                valve.set_high();
            }
            ValveState::Inactive => {
                valve.set_low();
            }
        }
    }
}

#[embassy_executor::task]
pub async fn check_main_arming(pin: Input<'static>) {
    let buzzer_armed_sender = BUZZER_WATCH.sender();
    loop {
        {
            let mut arming_state = MAIN_ARMING.lock().await;
            match pin.get_level() {
                Level::Low => {
                    *arming_state = ArmingState::Safe;
                    buzzer_armed_sender.send(BuzzerState::Idle);
                }
                Level::High => {
                    *arming_state = ArmingState::Armed;
                    buzzer_armed_sender.send(BuzzerState::Armed);
                }
            }
        }
        Timer::after(MAIN_ARMING_UPDATE_RATE).await;
    }
}
