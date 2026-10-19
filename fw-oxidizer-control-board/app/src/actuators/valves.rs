use crate::drivers::WATCH;
use crate::globals::STATE;
use datatypes::actuator::{NormallyClosedValve, NormallyOpenValve};
use embassy_futures::join::join;
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Receiver;

#[embassy_executor::task]
pub(crate) async fn valve_task(oxidizer_vent_valve: Output<'static>) {
    let oxidizer_vent_watcher = STATE.oxidizer_vent_control.receiver().unwrap();
    valve_task_impl(oxidizer_vent_valve, oxidizer_vent_watcher).await;
}

async fn valve_task_impl(
    mut valve: Output<'static>,
    mut watch: Receiver<'static, ThreadModeRawMutex, NormallyOpenValve, WATCH>,
) {
    loop {
        let state = watch.changed().await;
        match state {
            NormallyOpenValve::Closed => {
                valve.set_high();
            }
            NormallyOpenValve::Open => {
                valve.set_low();
            }
        }
    }
}

#[embassy_executor::task]
pub(crate) async fn main_valve_task(oss_ml_vlv: Output<'static>, fss_ml_vlv: Output<'static>) {
    let oss_ml_vlv_watcher = STATE.oxidizer_main_control.receiver().unwrap();
    let fss_ml_vlv_watcher = STATE.fuel_main_control.receiver().unwrap();
    let oss_ml_vlv_task = main_valve_task_impl(oss_ml_vlv, oss_ml_vlv_watcher);
    let fss_ml_vlv_task = main_valve_task_impl(fss_ml_vlv, fss_ml_vlv_watcher);

    join(oss_ml_vlv_task, fss_ml_vlv_task).await;
}

async fn main_valve_task_impl(
    mut valve: Output<'static>,
    mut watch: Receiver<'static, ThreadModeRawMutex, NormallyClosedValve, WATCH>,
) {
    loop {
        let state = watch.changed().await;
        match state {
            NormallyClosedValve::Open => {
                valve.set_high();
            }
            NormallyClosedValve::Closed => {
                valve.set_low();
            }
        }
    }
}
