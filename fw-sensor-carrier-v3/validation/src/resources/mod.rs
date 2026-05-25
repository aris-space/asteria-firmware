use embassy_stm32::{Peri, Peripherals, peripherals};

use crate::assign_resources;

pub mod buses;
pub mod buzzer;
pub mod flash;
pub mod leds;
pub mod sd;
pub mod sensors;
pub mod uart;

assign_resources! {
    buzzer: Buzzer {
        timer: TIM4,
        pin: PD15,
    }
    green_led: GreenLed {
        pin: PA10,
    }
    yellow_led: YellowLed {
        pin: PA9,
    }
    red_led: RedLed {
        pin: PA8,
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
    // Sensor block 2. With `bdma-bus2` it is I2C4 (BDMA, staged through SRAM4);
    // without it, the hardware-bridged I2C2 on general DMA.
    #[cfg(feature = "bdma-bus2")]
    bus2: Bus2 {
        periph: I2C4,
        scl: PF14,
        sda: PF15,
        tx_dma: BDMA_CH0,
        rx_dma: BDMA_CH1,
    }
    #[cfg(not(feature = "bdma-bus2"))]
    bus2: Bus2 {
        periph: I2C2,
        scl: PB10,
        sda: PB11,
        tx_dma: DMA2_CH0,
        rx_dma: DMA2_CH1,
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
    sd_card: SdCard {
        periph: SDMMC1,
        clk: PC12,
        cmd: PD2,
        d0: PC8,
        d1: PC9,
        d2: PC10,
        d3: PC11,
        detect: PD3,
        power: PD6,
    }
}

pub fn split(p: Peripherals) -> AssignedResources {
    split_resources!(p)
}
