//! Toggle LED based on GPIO input
//!
//! This will control an LED on GP25 based on a button hooked up to GP15. The button should be tied
//! to ground, as the input pin is pulled high internally by this example. When the button is
//! pressed, the LED will turn off.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use panic_halt as _;
use rp2040_hal::prelude::*;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = rp2040_pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = pac.IO_BANK0.split(pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut led_pin = pins.gpio25.into_output();
    let button_pin = pins.gpio15.into_input().pull_up();

    loop {
        if button_pin.is_high().unwrap() {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }
    }
}
