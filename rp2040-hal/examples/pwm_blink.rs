#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::PwmPin;
use panic_halt as _;
use rp2040_hal::{gpio::Pins, pwm::*, sio::Sio};

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

    // Init PWMs
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let mut pwm = pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Use B channel (which outputs to GPIO 25)
    let mut channel = pwm.channel_b;
    channel.output_to(pins.gpio25);
    loop {
        channel.set_duty(15000);
        // TODO: Replace with proper delays once we have clocks working
        cortex_m::asm::delay(5_000_000);
        channel.set_duty(30000);
        cortex_m::asm::delay(5_000_000);
    }
}
