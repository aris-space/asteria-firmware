use embassy_stm32::can::Can;
use embassy_stm32::{bind_interrupts, peripherals};

use super::CanBus;
use crate::tasks::can::RECEIVED_IDS;

impl CanBus {
    pub fn setup(self) -> Can<'static> {
        bind_interrupts!(struct CanIrqs {
            FDCAN1_IT0 => embassy_stm32::can::IT0InterruptHandler<peripherals::FDCAN1>;
            FDCAN1_IT1 => embassy_stm32::can::IT1InterruptHandler<peripherals::FDCAN1>;
        });

        can_utils::setup::setup_can(self.periph, self.rx, self.tx, CanIrqs, RECEIVED_IDS)
    }
}
