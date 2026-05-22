use defmt::{error, warn};
use embassy_stm32::can::CanRx;
use hermes_can::messages::Message;

use super::{CanReceiver, THIS_BOARD_ID};

#[embassy_executor::task]
pub async fn rx_task(mut can_rx: CanRx<'static>) -> ! {
    loop {
        match can_rx.recv().await {
            Ok((msg, _ts)) => match msg {
                Message::ResetAll(_) => {
                    warn!("CAN: ResetAll received, resetting");
                    cortex_m::peripheral::SCB::sys_reset();
                }
                Message::ResetSpecific(x) if x.board_id == THIS_BOARD_ID => {
                    warn!("CAN: ResetSpecific received, resetting");
                    cortex_m::peripheral::SCB::sys_reset();
                }
                _ => {}
            },
            Err(err) => error!("CAN RX error: {:?}", err),
        }
    }
}
