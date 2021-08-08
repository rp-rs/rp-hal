//! Blinks the LED on a Pico board
//!
//! This will fade in/out the LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::PwmPin;
use embedded_time::fixed_point::FixedPoint;
use panic_halt as _;
use pico::{
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        pwm::*,
        watchdog::Watchdog,
    },
    Pins, XOSC_CRYSTAL_FREQ,
};
use rp2040_hal::sio::Sio;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

const LOW: u16 = 0;
const HIGH: u16 = 25000;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

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

    let sio = Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Init PWMs
    let mut pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Use B channel (which outputs to GPIO 25)
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    loop {
        for i in (LOW..=HIGH).skip(100) {
            delay.delay_us(8);
            channel.set_duty(i);
        }

        for i in (LOW..=HIGH).rev().skip(100) {
            delay.delay_us(8);
            channel.set_duty(i);
        }

        delay.delay_ms(500);
    }
}
