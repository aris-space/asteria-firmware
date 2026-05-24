use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{bind_interrupts, peripherals};
use static_cell::StaticCell;

use super::Usb;

pub type UsbDriver = Driver<'static, peripherals::USB_OTG_HS>;

static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();

impl Usb {
    pub fn setup(self) -> UsbDriver {
        bind_interrupts!(struct UsbIrqs {
            OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
        });

        let ep_out_buffer = EP_OUT_BUFFER.init([0u8; 256]);
        let mut config = usb::Config::default();
        // Bus-powered; vbus_detection requires the pin wired, which this board
        // doesn't, so leave it off.
        config.vbus_detection = false;

        Driver::new_fs(
            self.periph,
            UsbIrqs,
            self.dp,
            self.dm,
            ep_out_buffer,
            config,
        )
    }
}
