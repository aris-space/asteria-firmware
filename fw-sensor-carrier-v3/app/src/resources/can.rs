use embassy_stm32::can::{BufferedCanFd, RxFdBuf, TxFdBuf};
use embassy_stm32::{bind_interrupts, peripherals};
use static_cell::StaticCell;

use super::CanBus;
use crate::tasks::can::{RECEIVED_IDS, RX_BUFFER_LEN, TX_BUFFER_LEN};

pub type BufferedCan = BufferedCanFd<'static, TX_BUFFER_LEN, RX_BUFFER_LEN>;

impl CanBus {
    pub fn setup(self) -> BufferedCan {
        bind_interrupts!(struct CanIrqs {
            FDCAN3_IT0 => embassy_stm32::can::IT0InterruptHandler<peripherals::FDCAN3>;
            FDCAN3_IT1 => embassy_stm32::can::IT1InterruptHandler<peripherals::FDCAN3>;
        });

        static TX_BUFFER: StaticCell<TxFdBuf<TX_BUFFER_LEN>> = StaticCell::new();
        static RX_BUFFER: StaticCell<RxFdBuf<RX_BUFFER_LEN>> = StaticCell::new();
        let can = can_utils::setup::setup_can(self.periph, self.rx, self.tx, CanIrqs, RECEIVED_IDS);
        // buffer right away, otherwise the rx fifo can fill up and jam
        can.buffered_fd(
            TX_BUFFER.init(TxFdBuf::new()),
            RX_BUFFER.init(RxFdBuf::new()),
        )
    }
}
