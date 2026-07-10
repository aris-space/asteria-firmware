use assign_resources::assign_resources;
use embassy_stm32::{Peri, Peripherals, peripherals};

pub mod buses;
pub mod can;
pub mod flash;
pub mod leds;
pub mod sensors;
pub mod uart;
pub mod usb;

assign_resources! {
    buzzer: Buzzer {
        pin: PD15,
    }
    green_led: GreenLed {
        pin: PA8,
    }
    yellow_led: YellowLed {
        pin: PA9,
    }
    red_led: RedLed {
        pin: PA10,
    }
    gps1_uart: Gps1Uart {
        periph: UART8,
        rx: PE0,
        tx: PE1,
        rx_dma: DMA1_CH0,
        tx_dma: DMA1_CH1,
    }
    gps2_uart: Gps2Uart {
        periph: UART7,
        rx: PE7,
        tx: PE8,
        rx_dma: DMA1_CH2,
        tx_dma: DMA1_CH3,
    }
    imu1: Imu1 {
        periph: SPI1,
        sck: PG11,
        mosi: PD7,
        miso: PG9,
        tx_dma: DMA1_CH4,
        rx_dma: DMA1_CH5,
        cs: PG10,
        int1: PE4,
        exti: EXTI4,
    }
    imu2: Imu2 {
        periph: SPI4,
        sck: PE12,
        mosi: PE14,
        miso: PE13,
        tx_dma: DMA1_CH6,
        rx_dma: DMA1_CH7,
        cs: PE11,
        int1: PE15,
        exti: EXTI15,
    }
    bus1: Bus1 {
        periph: I2C5,
        scl: PF1,
        sda: PF0,
        tx_dma: DMA2_CH2,
        rx_dma: DMA2_CH7,
    }
    bus2: Bus2 {
        periph: I2C2,
        scl: PB10,
        sda: PB11,
        tx_dma: DMA2_CH0,
        rx_dma: DMA2_CH1,
    }
    can_bus: CanBus {
        periph: FDCAN3,
        rx: PF6,
        tx: PF7,
    }
    flash: Flash {
        periph: OCTOSPI1,
        sck: PB2,
        io0: PB1,
        io1: PB0,
        hold: PA6,
        wp: PA7,
        ncs: PG6,
    }
    usb: Usb {
        periph: USB_OTG_HS,
        dp: PA12,
        dm: PA11,
    }
}

pub fn split(p: Peripherals) -> AssignedResources {
    split_resources!(p)
}
