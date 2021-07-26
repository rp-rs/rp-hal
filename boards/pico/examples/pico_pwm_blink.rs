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
    XOSC_CRYSTAL_FREQ,
};

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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, *clocks.system_clock.freq().integer());

    let mut pwm_pin = Pwm4::new(25);

    //Instead of having it take references to all of these pac objects, eventually this should just
    //take ownership of a GPIO pin.
    pwm_pin.default_config(
        &mut pac.PWM,
        &mut pac.PADS_BANK0,
        &mut pac.IO_BANK0,
        &mut pac.RESETS,
    );

    pwm_pin.set_ph_correct();

    pwm_pin.enable();

    loop {
        for i in (LOW..=HIGH).skip(100) {
            delay.delay_us(8);
            pwm_pin.set_duty(i);
        }

        for i in (LOW..=HIGH).rev().skip(100) {
            delay.delay_us(8);
            pwm_pin.set_duty(i);
        }

        delay.delay_ms(500);
    }
}
