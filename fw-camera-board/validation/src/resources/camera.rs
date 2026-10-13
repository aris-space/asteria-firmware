//! RunCam Split power switch and control UART. The camera speaks the RunCam
//! Device Protocol at [`BAUD_RATE`].

use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart};
use embassy_stm32::{bind_interrupts, dma, peripherals};

use super::{CameraPower, CameraUart};

pub const BAUD_RATE: u32 = 115_200;

impl CameraPower {
    /// Starts with the camera off.
    pub fn setup(self) -> Output<'static> {
        Output::new(self.pin, Level::Low, Speed::Low)
    }
}

impl CameraUart {
    pub fn setup(self) -> Uart<'static, Async> {
        bind_interrupts!(struct CameraIrqs {
            USART1 => usart::InterruptHandler<peripherals::USART1>;
            DMA1_CHANNEL1 => dma::InterruptHandler<peripherals::DMA1_CH1>;
            DMA1_CHANNEL2 => dma::InterruptHandler<peripherals::DMA1_CH2>;
        });

        let mut config = usart::Config::default();
        config.baudrate = BAUD_RATE;

        Uart::new(
            self.periph,
            self.rx,
            self.tx,
            self.tx_dma,
            self.rx_dma,
            CameraIrqs,
            config,
        )
        .expect("Failed to create camera UART")
    }
}
