//! Camera power. The camera stays off until it can be switched over CAN.

use core::future::pending;

use embassy_stm32::gpio::Output;

#[embassy_executor::task]
pub async fn task(_power: Output<'static>) {
    pending::<()>().await;
}
