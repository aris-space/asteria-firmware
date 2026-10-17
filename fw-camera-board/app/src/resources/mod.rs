use assign_resources::assign_resources;
use embassy_stm32::{Peri, Peripherals, peripherals};

pub mod address;
pub mod camera;
pub mod can;
pub mod leds;

assign_resources! {
    debug_led: DebugLed {
        pin: PA4,
    }
    address: Address {
        add0: PA0,
        add1: PA1,
        add2: PA2,
        add3: PA3,
    }
    camera_power: CameraPower {
        pin: PA15,
    }
    camera_uart: CameraUart {
        periph: USART1,
        tx: PA9,
        rx: PA10,
        tx_dma: DMA1_CH1,
        rx_dma: DMA1_CH2,
    }
    can_bus: CanBus {
        periph: FDCAN1,
        rx: PA11,
        tx: PA12,
    }
}

pub fn split(p: Peripherals) -> AssignedResources {
    split_resources!(p)
}
