use core::mem::MaybeUninit;
use stm32f4xx_hal::otg_fs::{UsbBus, UsbBusType};
pub use stm32f4xx_hal::otg_fs::USB;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::UsbDevice;
use usb_device::prelude::*;
use usbd_serial::SerialPort;
pub use usbd_serial::embedded_io::Write;

pub type Device = UsbDevice<'static, UsbBusType>;
pub type Port = SerialPort<'static, UsbBusType>;

// Resources
pub type UsbBusStore = MaybeUninit<UsbBusAllocator<UsbBusType>>;
pub type EpMemory = [u32; 1024];

#[inline]
pub const fn init_usb_bus_store() -> UsbBusStore {
    MaybeUninit::uninit()
}

#[inline]
pub const fn init_ep_memory() -> EpMemory {
    [0; 1024]
}

// Init
pub fn init(usb: USB, ep_memory: &'static mut EpMemory, usb_bus_store: &'static mut UsbBusStore) -> (Device, Port) {
    let usb_bus = usb_bus_store.write(UsbBus::new(usb, ep_memory));

    let usb_serial = SerialPort::new(usb_bus);

    let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .device_class(usbd_serial::USB_CLASS_CDC)
        .strings(&[StringDescriptors::default()
            .manufacturer("Fake Company")
            .product("Product")
            .serial_number("TEST")])
        .unwrap()
        .build();

    (usb_dev, usb_serial)

}