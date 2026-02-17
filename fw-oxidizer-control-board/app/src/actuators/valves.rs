use crate::drivers::WATCH;
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use hermes_can::messages::board_status::ValveState;

pub static OXD_VENT_CONTROL: Watch<ThreadModeRawMutex, ValveState, WATCH> = Watch::new();

#[embassy_executor::task]
pub(crate) async fn valve_task(oxd_vnt_vlv: Output<'static>) {
    let oxd_vnt_watcher = OXD_VENT_CONTROL.receiver().unwrap();
    let oxd_vnt_task = valve_task_impl(oxd_vnt_vlv, oxd_vnt_watcher);

    oxd_vnt_task.await;
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
