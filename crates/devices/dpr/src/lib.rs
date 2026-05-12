mod dpr;
mod pid;

use crate::dpr::DprLoopState::{ActiveNominal, Passive};
use crate::dpr::{DprLoopState, DPR};
use crate::pid::PIDGain;
use datatypes::actuator::DPRValve;
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Sender;
use embassy_sync::watch::Receiver;

pub type DPRPressure = f32;

#[embassy_executor::task]
pub async fn pid_controller(
    mut valve_pin: Output<'static>,
    control_receiver: Receiver<ThreadModeRawMutex, DPRValve, 5>,
    pressure_receiver: Receiver<ThreadModeRawMutex, DPRPressure, 5>,
    gain_receiver: Receiver<ThreadModeRawMutex, PIDGain, 5>,
    status_sender : Sender<ThreadModeRawMutex, DprLoopState, 5>
) {
    let control_receiver = control_receiver.receiver().unwrap();
    let control_sender = control_receiver.sender();

    let mut dpr = DPR::new(valve_pin).await;

    loop {
        if let Some(cfg) = control_receiver.try_changed() {
            match cfg {
                DPRValve::Enabled { setpoint: stp } => {
                    dpr.update_state(stp, ActiveNominal);
                    dpr.reset();
                }
                DPRValve::Disabled => {
                    dpr.loop_state = Passive;
                }
            }
        }

        if let Some(p) = pressure_receiver.try_changed() {
            dpr.update_pressure(p);
        }

        dpr.check_overpressure();

        dpr.update_valve_state(&mut valve_pin);

        dpr.tick().await;
    }
}
