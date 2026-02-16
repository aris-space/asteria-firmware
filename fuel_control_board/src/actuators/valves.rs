use crate::drivers::WATCH;
use core::future::pending;
use embassy_futures::join::join;
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use hermes_can::messages::board_status::ValveState;

pub static PRZ_VENT_CONTROL: Watch<ThreadModeRawMutex, ValveState, WATCH> = Watch::new();
pub static FSS_VENT_CONTROL: Watch<ThreadModeRawMutex, ValveState, WATCH> = Watch::new();

#[embassy_executor::task]
pub(crate) async fn valve_task(prz_vnt_vlv: Output<'static>, fue_vnt_vlv: Output<'static>) {
    let prz_vnt_watcher = PRZ_VENT_CONTROL.receiver().unwrap();
    let fue_vnt_watcher = FSS_VENT_CONTROL.receiver().unwrap();
    let prz_vnt_task = valve_task_impl(prz_vnt_vlv, prz_vnt_watcher);
    let fue_vnt_task = valve_task_impl(fue_vnt_vlv, fue_vnt_watcher);

    join(prz_vnt_task, fue_vnt_task).await;

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
