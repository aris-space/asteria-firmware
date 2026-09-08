#![no_std]
pub mod dpr;
pub mod pid;

use crate::dpr::DPR;
use datatypes::actuator::DPRValve;
use datatypes::status::DprGainInfo;
use datatypes::status::DprLoopInfo;
use embassy_stm32::gpio::Output;
#[cfg(any(doc, docsrs))]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as DprRawMutex;
#[cfg(not(any(doc, docsrs)))]
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex as DprRawMutex;
use embassy_sync::watch::{Receiver, Sender};
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn pid_controller(
    valve_pin: Output<'static>,
    control_receiver: Receiver<'static, DprRawMutex, DPRValve, 5>,
    pressure_receiver: Receiver<'static, DprRawMutex, f32, 5>,
    pid_receiver: Receiver<'static, DprRawMutex, DprGainInfo, 5>,
    status_sender: Sender<'static, DprRawMutex, DprLoopInfo, 5>,
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

        // Apply any gains received over CAN. step_bang() only uses the setpoint,
        // but the values are kept current so the PID path stays tunable.
        dpr.update_pid();

        // Update pressure if available
        dpr.update_pressure();

        // Check for overpressure
        dpr.handle_overpressure();

        // Compute controller output and actuate valve
        dpr.step_bang().await;

        Timer::after_micros(1600).await;
    }
}
