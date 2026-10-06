//! Receive-only GNSS UARTs, both receivers are configured ahead of time to send
//! UBX packets at 921600 baud. This is done so that we can save DMA channels and use them
//! where they are actually needed.

use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, UartRx};
use embassy_stm32::{bind_interrupts, peripherals};

use super::{Gps1Uart, Gps2Uart};

fn config() -> usart::Config {
    let mut config = usart::Config::default();
    config.data_bits = usart::DataBits::DataBits8;
    config.parity = usart::Parity::ParityNone;
    config.stop_bits = usart::StopBits::STOP1;
    config.baudrate = 921_600;
    config
}

impl Gps1Uart {
    pub fn setup(self) -> UartRx<'static, Async> {
        bind_interrupts!(struct Gps1Irqs {
            UART8 => usart::InterruptHandler<peripherals::UART8>;
            DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
        });
        UartRx::new(self.periph, self.rx, self.rx_dma, Gps1Irqs, config())
            .expect("Failed to create GPS1 UART")
    }
}

impl Gps2Uart {
    pub fn setup(self) -> UartRx<'static, Async> {
        bind_interrupts!(struct Gps2Irqs {
            UART7 => usart::InterruptHandler<peripherals::UART7>;
            DMA1_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH2>;
        });
        UartRx::new(self.periph, self.rx, self.rx_dma, Gps2Irqs, config())
            .expect("Failed to create GPS2 UART")
    }
}
