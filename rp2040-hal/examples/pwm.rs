#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::PwmPin;
use panic_halt as _;
use rp2040_hal::prelude::*;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = rp2040_pac::Peripherals::take().unwrap();

    let pins = pac.PWM.split(pac.PADS_BANK0, pac.IO_BANK0, &mut pac.RESETS);

    let mut pwm_pin = pins.pwm2.default_config();

    pwm_pin.set_ph_correct();

    pwm_pin.enable();

    loop {
        pwm_pin.set_duty(15000);
        // TODO: Replace with proper delays once we have clocks working
        cortex_m::asm::delay(5_000_000);
        pwm_pin.set_duty(30000);
        cortex_m::asm::delay(5_000_000);
    }
}
