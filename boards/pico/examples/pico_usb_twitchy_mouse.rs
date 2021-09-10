//! This is a port of
//! https://github.com/atsamd-rs/atsamd/blob/master/boards/itsybitsy_m0/examples/twitching_usb_mouse.rs
#![no_std]
#![no_main]

use cortex_m::interrupt::free as disable_interrupts;
use cortex_m_rt::entry;
use embedded_time::fixed_point::FixedPoint;
use panic_halt as _;
use pico::{
    hal::{
        self,
        clocks::{init_clocks_and_plls, Clock},
        pac::{self, interrupt},
        usb::UsbBus,
        watchdog::Watchdog,
    },
    XOSC_CRYSTAL_FREQ,
};
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::MouseReport;
use usbd_hid::hid_class::HIDClass;

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

    let usb_hid = HIDClass::new(
        unsafe { USB_BUS.as_ref().unwrap() },
        MouseReport::desc(),
        60,
    );
    unsafe {
        USB_HID = Some(usb_hid);
    }

    let usb_dev = UsbDeviceBuilder::new(
        unsafe { USB_BUS.as_ref().unwrap() },
        UsbVidPid(0x16c0, 0x27dd),
    )
    .manufacturer("Fake company")
    .product("Twitchy Mousey")
    .serial_number("TEST")
    .device_class(0xEF) // misc
    .build();
    unsafe {
        USB_DEVICE = Some(usb_dev);
    }

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
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    loop {
        delay.delay_ms(100);
        push_mouse_movement(MouseReport {
            x: 0,
            y: 4,
            buttons: 0,
            wheel: 0,
        })
        .ok()
        .unwrap_or(0);
        delay.delay_ms(100);
        push_mouse_movement(MouseReport {
            x: 0,
            y: -4,
            buttons: 0,
            wheel: 0,
        })
        .ok()
        .unwrap_or(0);
    }
}

static mut USB_DEVICE: Option<UsbDevice<UsbBus>> = None;
static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_HID: Option<HIDClass<UsbBus>> = None;

fn push_mouse_movement(report: MouseReport) -> Result<usize, usb_device::UsbError> {
    disable_interrupts(|_| unsafe { USB_HID.as_mut().map(|hid| hid.push_input(&report)) }).unwrap()
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    usb_dev.poll(&mut [usb_hid]);

    // Clear pending interrupt flags
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
