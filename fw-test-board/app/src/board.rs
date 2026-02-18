use core::fmt::Debug;
use defmt_brtt::DefmtConsumer;
use embassy_executor::InterruptExecutor;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::once_lock::OnceLock;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::Spi;
use embassy_stm32::time::mhz;
use embassy_time::{Delay, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use static_cell::StaticCell;
use w25q256jv::W25q256jv;

type FlashSpi = Spi<'static, Blocking>;
type FlashDevice = ExclusiveDevice<FlashSpi, Output<'static>, Delay>;
pub type BoardFlash = W25q256jv<FlashDevice, Output<'static>, Output<'static>>;
pub type FlashAdapter<'a> = w25q256jv::W25q256jvLfsStorage<
    'a,
    FlashDevice,
    Output<'static>,
    Output<'static>,
    typenum::U4096,
    typenum::U512,
>;

pub static INTERRUPT_EXECUTOR: InterruptExecutor = InterruptExecutor::new();
pub static BOARD: OnceLock<Mutex<CriticalSectionRawMutex, Board>> = OnceLock::new();
static BOARD_FLASH: StaticCell<BoardFlash> = StaticCell::new();

#[interrupt]
unsafe fn TIM2() {
    // Safety: this handler is bound to the same IRQ used to start this executor.
    unsafe { INTERRUPT_EXECUTOR.on_interrupt() };
}

pub struct Board {
    pub yellow: Option<Output<'static>>,
    pub green: Option<Output<'static>>,
    pub red: Option<Output<'static>>,
    pub flash: Option<FlashAdapter<'static>>,
    pub defmt_log: Option<DefmtConsumer>,
}

impl Debug for Board {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Board")
            .field("yellow", &self.yellow.is_some())
            .field("green", &self.green.is_some())
            .field("red", &self.red.is_some())
            .field("flash", &self.flash.is_some())
            .field("defmt_log", &self.defmt_log.is_some())
            .finish()
    }
}

impl Board {
    pub fn new(p: embassy_stm32::Peripherals) -> Self {
        let defmt_consumer = defmt_brtt::init().expect("failed to initialize logger");

        // Configure LEDs
        let green = Output::new(p.PB0, Level::Low, Speed::Low);
        let yellow = Output::new(p.PB1, Level::Low, Speed::Low);
        let red = Output::new(p.PB2, Level::Low, Speed::Low);

        // SPI2 for W25Q256JV flash (SCK=PB13, MISO=PB14, MOSI=PB15, CS=PC7)
        let mut spi_config = embassy_stm32::spi::Config::default();
        spi_config.frequency = mhz(50);
        spi_config.rise_fall_speed = Speed::VeryHigh;
        let spi = Spi::new_blocking(p.SPI2, p.PB13, p.PB15, p.PB14, spi_config);
        let cs = Output::new(p.PC7, Level::High, Speed::VeryHigh);
        let device = ExclusiveDevice::new(spi, cs, Delay).expect("spi exclusive");
        let hold = Output::new(p.PC14, Level::High, Speed::Low);
        let wp = Output::new(p.PC15, Level::High, Speed::Low);
        let flash = W25q256jv::new(device, hold, wp).expect("w25q256jv init");
        let flash = BOARD_FLASH.init(flash);
        let flash_adapter = FlashAdapter::new(flash);

        Self {
            yellow: Some(yellow),
            flash: Some(flash_adapter),
            green: Some(green),
            red: Some(red),
            defmt_log: Some(defmt_consumer),
        }
    }
}

#[embassy_executor::task]
pub async fn blink(mut led: Output<'static>) {
    loop {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(900).await;
    }
}
