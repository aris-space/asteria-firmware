use embassy_stm32::mode::Async;
use embassy_stm32::usart::{self, Uart};
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
    pub fn setup(self) -> Uart<'static, Async> {
        let config = config();
        bind_interrupts!(struct Gps1Irqs {
            UART8 => usart::InterruptHandler<peripherals::UART8>;
            DMA1_STREAM0 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH0>;
            DMA1_STREAM1 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH1>;
        });

        Uart::new(
            self.periph,
            self.rx,
            self.tx,
            self.tx_dma,
            self.rx_dma,
            Gps1Irqs,
            config,
        )
        .expect("Failed to create GPS1 data UART")
    }
}

impl Gps2Uart {
    pub fn setup(self) -> Uart<'static, Async> {
        let config = config();
        bind_interrupts!(struct Gps2Irqs {
            UART7 => usart::InterruptHandler<peripherals::UART7>;
            DMA1_STREAM2 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH2>;
            DMA1_STREAM3 => embassy_stm32::dma::InterruptHandler<peripherals::DMA1_CH3>;
        });

        Uart::new(
            self.periph,
            self.rx,
            self.tx,
            self.tx_dma,
            self.rx_dma,
            Gps2Irqs,
            config,
        )
        .expect("Failed to create GPS2 data UART")
    }
}
