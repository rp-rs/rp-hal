//! # Seeeduino XIAO RP2040 Blinky Example
//!
//! Blinks the LED on a Seeeduino XIAO RP2040 16MB board.
//!
//! This will blink an LED attached to GPIO25, which is the pin the XIAO RP2040
//! uses for the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// The macro for our start-up function
use seeeduino_xiao_rp2040::entry;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::PwmPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use seeeduino_xiao_rp2040::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use seeeduino_xiao_rp2040::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use seeeduino_xiao_rp2040::hal;

use hal::gpio::PinState;

// The minimum PWM value (i.e. LED brightness) we want
const LOW: u16 = 0;

// The maximum PWM value (i.e. LED brightness) we want
const HIGH: u16 = 60000;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        seeeduino_xiao_rp2040::XOSC_CRYSTAL_FREQ,
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
    let pins = seeeduino_xiao_rp2040::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm0;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM0 to the red LED pin, initially off
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led_red);
    channel.set_duty(u16::MAX);

    // Set the blue LED to be an output, initially off
    let mut led_blue_pin = pins.led_blue.into_push_pull_output_in_state(PinState::High);

    // Turn off the green LED
    let mut _led_green_pin = pins
        .led_green
        .into_push_pull_output_in_state(PinState::High);

    loop {
        // Blink blue LED at 1 Hz
        for _ in 0..5 {
            led_blue_pin.set_low().unwrap();
            delay.delay_ms(500);
            led_blue_pin.set_high().unwrap();
            delay.delay_ms(500);
        }

        // Ramp red LED brightness up
        for i in (LOW..=HIGH).skip(30) {
            delay.delay_us(100);
            channel.set_duty(u16::MAX - i);
        }

        // Ramp red LED brightness down
        for i in (LOW..=HIGH).rev().skip(30) {
            delay.delay_us(100);
            channel.set_duty(u16::MAX - i);
        }
    }
}

// End of file
