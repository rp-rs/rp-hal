//! # Pico Countdown Blinky Example
//!
//! Blinks the LED on a Pico board, using an RP2040 Timer in Count-down mode.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use rp_pico::entry;

use cortex_m::prelude::*;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Traits for converting integers to amounts of time
use embedded_time::duration::Extensions;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let _clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // Configure the Timer peripheral in count-down mode
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut count_down = timer.count_down();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    // Blink the LED at 1 Hz
    loop {
        // LED on, and wait for 500ms
        led_pin.set_high().unwrap();
        count_down.start(500.milliseconds());
        let _ = nb::block!(count_down.wait());

        // LED off, and wait for 500ms
        led_pin.set_low().unwrap();
        count_down.start(500.milliseconds());
        let _ = nb::block!(count_down.wait());
    }
}
