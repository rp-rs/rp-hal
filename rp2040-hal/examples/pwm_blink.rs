#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use rp2040_hal::{
    gpio::{pin::*, Pins},
    pwm::{Pwm, PwmOutput},
    sio::Sio,
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = rp2040_pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);

    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    pins.gpio25.into_mode::<FunctionPwm>();

    let pwm = Pwm::new(pac.PWM, &mut pac.RESETS);

    let pwm4 = pwm.slice(4).unwrap();
    pwm4.default_config();
    pwm4.enable();

    let pwm4b = pwm4.channel(PwmOutput::B);
    loop {
        pwm4b.set_level(0x8000);
        // TODO: Replace with proper delays once we have clocks working
        cortex_m::asm::delay(30_000_000);
        pwm4b.set_level(0xffff);
        cortex_m::asm::delay(30_000_000);
    }
}
