//! Creates a USB Serial device on a Pico board, with the USB driver running in the USB interrupt
//!
//! This will create a USB Serial device echoing anything it receives converting to caps the ASCII
//! alphabetical caracters.
#![no_std]
#![no_main]

use crate::pac::interrupt;
use cortex_m_rt::entry;
use panic_halt as _;
use pico::{
    hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        pac,
        sio::Sio,
        usb::UsbBus,
        watchdog::Watchdog,
    },
    XOSC_CRYSTAL_FREQ,
};
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

// Static data so that it can be accessed in both main and interrupt context
static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut SAID_HELLO: bool = false;

// Blinky-related imports, not needed for USB
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use pico::Pins;

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

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        USB_BUS = Some(usb_bus);
    }

    let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap() });
    unsafe {
        USB_SERIAL = Some(serial);
    }

    let usb_dev = UsbDeviceBuilder::new(
        unsafe { USB_BUS.as_ref().unwrap() },
        UsbVidPid(0x16c0, 0x27dd),
    )
    .manufacturer("Fake company")
    .product("Serial port")
    .serial_number("TEST")
    .max_packet_size_0(64)
    .device_class(2) // from: https://www.usb.org/defined-class-codes
    .build();
    unsafe {
        USB_DEVICE = Some(usb_dev);
    }

    // The USB driver doesn't enable key interrupts yet, so manually do that here
    unsafe {
        let p = pac::Peripherals::steal();
        // Enable interrupts for when a buffer is done, when the bus is reset,
        // and when a setup packet is received
        p.USBCTRL_REGS.inte.modify(|_, w| {
            w.buff_status()
                .set_bit()
                .bus_reset()
                .set_bit()
                .setup_req()
                .set_bit()
        });
    }
    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    // No more USB code after this point in main!
    // We can do anything we want in here since USB is handled
    // in the interrupt - let's blink an LED.
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.led.into_push_pull_output();
    loop {
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    let mut buf = [0u8; 64];
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    if !SAID_HELLO {
        SAID_HELLO = true;
        let _ = serial.write(b"HelloWorld!\r\n");
    }

    if usb_dev.poll(&mut [serial]) {
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

    // Clear pending interrupt flags here.
    // We could also move some of our code into these states to handle events
    let p = pac::Peripherals::steal();
    let status = &p.USBCTRL_REGS.sie_status;
    if status.read().ack_rec().bit_is_set() {
        status.modify(|_r, w| w.ack_rec().set_bit());
    }
    if status.read().setup_rec().bit_is_set() {
        status.modify(|_r, w| w.setup_rec().set_bit());
    }
    if status.read().trans_complete().bit_is_set() {
        status.modify(|_r, w| w.trans_complete().set_bit());
    }
    if status.read().bus_reset().bit_is_set() {
        status.modify(|_r, w| w.bus_reset().set_bit());
    }
}
