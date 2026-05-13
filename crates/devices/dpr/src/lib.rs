#![no_std]
pub mod dpr;
pub mod pid;

use crate::dpr::DPR;
use datatypes::actuator::DPRValve;
use datatypes::status::DprGainInfo;
use datatypes::status::DprLoopInfo::{self, *};
use embassy_stm32::gpio::Output;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Sender, Watch};

#[embassy_executor::task]
pub async fn pid_controller(
    valve_pin: Output<'static>,
    control_receiver: Receiver<'static, ThreadModeRawMutex, DPRValve, 5>,
    pressure_receiver: Receiver<'static, ThreadModeRawMutex, f32, 5>,
    pid_receiver: Receiver<'static, ThreadModeRawMutex, DprGainInfo, 5>,
    status_sender: Sender<'static, ThreadModeRawMutex, DprLoopInfo, 5>,
) {
    let mut dpr = DPR::new(
        valve_pin,
        control_receiver,
        pressure_receiver,
        pid_receiver,
        status_sender,
    );

    loop {
        // Check if config has changed
        dpr.update_state();

        // Check if gain has changed
        dpr.update_pid();

        // Update pressure if available
        dpr.update_pressure();

        // Check for overpressure
        dpr.handle_overpressure();
        // ToDo: implement state update send (maybe)

        // Compute controller output and actuate valve
        dpr.step().await;
    }
}
