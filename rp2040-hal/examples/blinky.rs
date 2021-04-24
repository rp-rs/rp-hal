//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

use embedded_hal::digital::v2::OutputPin;
use rp2040_hal::prelude::*;

#[entry]
fn main() -> ! {
    let mut pac = rp2040_pac::Peripherals::take().unwrap();

    let pins = pac.IO_BANK0.split(pac.PADS_BANK0, pac.SIO, &mut pac.RESETS);
    let mut led_pin = pins.gpio25.into_output();

    loop {
        led_pin.set_low().unwrap();
        // TODO: I dare not use delays until we've got clocks running
        led_pin.set_high().unwrap();
        // TODO: Other delay
    }
}
