use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::{Peri, bind_interrupts, dma, peripherals};

use super::Camera;
use crate::runcam::RunCam;

pub const BAUD_RATE: u32 = 115_200;

bind_interrupts!(struct CameraIrqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
    DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
    DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
});

impl Camera {
    pub fn setup(self) -> RunCam {
        // PA15 is pulled up after reset (JTDI), so set it low to keep the camera off.
        let power = Output::new(self.power, Level::Low, Speed::Low);
        let uart = CameraUart {
            periph: self.periph,
            tx: self.tx,
            rx: self.rx,
            tx_dma: self.tx_dma,
            rx_dma: self.rx_dma,
        };
        RunCam::new(power, uart)
    }
}

/// The UART only exists while the camera is powered. Dropping it puts TX back to
/// analog, otherwise the idle-high TX back-powers the camera and it never resets.
pub struct CameraUart {
    periph: Peri<'static, peripherals::USART1>,
    tx: Peri<'static, peripherals::PA9>,
    rx: Peri<'static, peripherals::PA10>,
    tx_dma: Peri<'static, peripherals::DMA1_CH1>,
    rx_dma: Peri<'static, peripherals::DMA1_CH2>,
}

impl CameraUart {
    pub fn open(&mut self) -> Uart<'_, Async> {
        let mut config = usart::Config::default();
        config.baudrate = BAUD_RATE;

        Uart::new(
            self.periph.reborrow(),
            self.rx.reborrow(),
            self.tx.reborrow(),
            self.tx_dma.reborrow(),
            self.rx_dma.reborrow(),
            CameraIrqs,
            config,
        )
        .expect("Failed to create camera UART")
    }
}
