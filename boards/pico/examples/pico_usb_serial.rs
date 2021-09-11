//! Creates a USB Serial device on a Pico board.
//!
//! This will create a USB Serial device echoing anything it receives converting to caps the ASCII
//! alphabetical caracters.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use pico::{
    hal::{clocks::init_clocks_and_plls, pac, timer::Timer, usb::UsbBus, watchdog::Watchdog},
    XOSC_CRYSTAL_FREQ,
};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let timer = Timer::new(pac.TIMER);
    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();

    let mut said_hello = false;
    loop {
        if !said_hello && timer.get_counter() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"HelloWorld!\r\n");
        }

        if !usb_dev.poll(&mut [&mut serial]) {
            continue;
        }

        let mut buf = [0u8; 64];
        let _ = serial.read(&mut buf).map(|count| {
            if count == 0 {
                return;
            }

            // Echo back in upper case
            buf.iter_mut().take(count).for_each(|c| {
                if let 0x61..=0x7a = *c {
                    *c &= !0x20;
                }
            });

            let mut wr_ptr = &buf[..count];
            while !wr_ptr.is_empty() {
                let _ = serial.write(wr_ptr).map(|len| {
                    wr_ptr = &wr_ptr[len..];
                });
            }
        });
    }
}
