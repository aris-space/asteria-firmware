use embassy_stm32::peripherals::USB_OTG_HS;
use embassy_stm32::{bind_interrupts, usb as embassy_usb};
use static_cell::StaticCell;
use super::{Usb, UsbDriver};

pub type UsbDriver = embassy_stm32::usb::Driver<'static, USB_OTG_HS>;

pub(super) static USB_EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();

bind_interrupts!(pub struct UsbIrqs {
    OTG_HS => embassy_usb::InterruptHandler<USB_OTG_HS>;
});

impl Usb {
    pub fn setup(self) -> UsbDriver {
        let buffer = USB_EP_OUT_BUFFER.init([0u8; 256]);
        embassy_stm32::usb::Driver::new_hs(
            self.usb,
            UsbIrqs,
            self.dp,
            self.dm,
            buffer,
            embassy_stm32::usb::Config::default(),
        )
    }
}
