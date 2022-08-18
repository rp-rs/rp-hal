//! # Badger2040 Blinky Example
//!
//! Blinks the activity LED on a badger2040 board, using an RP2040 Timer in Count-down mode.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use pimoroni_badger2040::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use pimoroni_badger2040::hal::pac;
use pimoroni_badger2040::hal::Timer;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use pimoroni_badger2040::hal;

// A few traits required for using the CountDown timer
use embedded_hal::timer::CountDown;
use fugit::ExtU32;

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
        pimoroni_badger2040::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = pimoroni_badger2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure the timer peripheral to be a CountDown timer for our blinky delay
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut delay = timer.count_down();

    // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    // Blink the LED at 1 Hz
    loop {
        // LED on, and wait for 500ms
        led_pin.set_high().unwrap();
        delay.start(500.millis());
        let _ = nb::block!(delay.wait());

        // LED off, and wait for 500ms
        led_pin.set_low().unwrap();
        delay.start(500.millis());
        let _ = nb::block!(delay.wait());
    }
}
