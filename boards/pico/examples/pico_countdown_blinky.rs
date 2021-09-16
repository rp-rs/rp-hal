//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_timer_CountDown;
use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::duration::Extensions;
use panic_halt as _;
use pico::{
    hal::{self as hal, clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog},
    XOSC_CRYSTAL_FREQ,
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let _clocks = init_clocks_and_plls(
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

    let timer = hal::timer::Timer::new(pac.TIMER);
    let mut count_down = timer.count_down();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led_pin = pins.gpio25.into_push_pull_output();

    loop {
        led_pin.set_high().unwrap();
        // wait for 500ms
        count_down.start(500.milliseconds());
        let _ = nb::block!(count_down.wait());

        led_pin.set_low().unwrap();
        // wait for 500ms
        count_down.start(500.milliseconds());
        let _ = nb::block!(count_down.wait());
    }
}
