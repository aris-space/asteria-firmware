use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::{bind_interrupts, dma, peripherals};

use super::Camera;
use crate::runcam::RunCam;

pub const BAUD_RATE: u32 = 115_200;

impl Camera {
    pub fn setup(self) -> RunCam {
        // PA15 is pulled up after reset (JTDI), so set it low to keep the camera off.
        let power = Output::new(self.power, Level::Low, Speed::Low);

        bind_interrupts!(struct CameraIrqs {
            USART1 => usart::InterruptHandler<peripherals::USART1>;
            DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
            DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
        });

        let mut config = usart::Config::default();
        config.baudrate = BAUD_RATE;

        let uart = Uart::new(
            self.periph,
            self.rx,
            self.tx,
            self.tx_dma,
            self.rx_dma,
            CameraIrqs,
            config,
        )
        .expect("Failed to create camera UART");

        RunCam::new(power, uart)
    }
}
