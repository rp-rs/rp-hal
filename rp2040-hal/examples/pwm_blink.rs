#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::PwmPin;
use panic_halt as _;
use rp2040_hal::pwm::*;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = rp2040_pac::Peripherals::take().unwrap();

    let mut pwm_pin = Pwm0::new(0);

    pwm_pin.default_config(&mut pac.PWM, &mut pac.PADS_BANK0, &mut pac.IO_BANK0, &mut pac.RESETS);

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
