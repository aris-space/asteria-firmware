#![allow(dead_code)]

use embassy_stm32::can::Can;
use embassy_stm32::{bind_interrupts, peripherals};

use super::CanBus;

impl CanBus {
    pub fn setup(self) -> Can<'static> {
        bind_interrupts!(struct CanIrqs {
            FDCAN3_IT0 => embassy_stm32::can::IT0InterruptHandler<peripherals::FDCAN3>;
            FDCAN3_IT1 => embassy_stm32::can::IT1InterruptHandler<peripherals::FDCAN3>;
        });

        todo!()
    }
}
