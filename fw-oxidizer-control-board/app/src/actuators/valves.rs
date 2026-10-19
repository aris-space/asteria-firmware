use crate::drivers::WATCH;
use crate::globals::STATE;
use datatypes::actuator::NormallyOpenValve;
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
