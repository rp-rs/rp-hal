//! # GPIO Dynamic Pin Type Mode Example
//!
//! This application demonstrates how to put GPIO pins into their Dynamic Pin type on the RP2040.
//!
//! Usually, the type of each pin is different (which allows the type system to catch misuse).
//! But this stops you storing the pins in an array, or allowing a struct to take any pin.
//! This mode is also referred to as "Erased", "Downgraded", "Degraded", or "Dynamic".
//!
//! In order to see the result of this program, you will need to put LEDs and a current limiting
//! resistor on each of GPIO 2, 3, 4, 5.  
//! The other side of the LED + resistor pair should be connected to GND.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use hal::gpio::{DynPinId, FunctionSioOutput, Pin, PullNone, PullUp};
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // We will use the RP2040 timer peripheral as our delay source
    let mut timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // To put pins into an array we have to convert them to Dynamically Typed pins.
    // This means they'll carry their pin and bank numbers around with them at run time,
    // rather than relying on the Type of the pin to track that.
    let mut pinarray: [Pin<DynPinId, FunctionSioOutput, PullNone>; 4] = [
        pins.gpio2
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio3
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio4
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
        pins.gpio5
            .into_push_pull_output()
            .into_pull_type()
            .into_dyn_pin(),
    ];

    // Also set a pin as a dynamic input. We won't use this, it is just to demonstrate that
    // pins can have other functions and still be Dynamically typed.
    let _in_pin = pins.gpio23.into_floating_input().into_dyn_pin();

    // You can also let the target type set the pin mode, using the type system to guide it.
    // Once again, we're not going to use this array. The only reason it is here is to demonstrate a less verbose way to set pin modes
    let mut _type_coerce: [Pin<DynPinId, FunctionSioOutput, PullUp>; 1] =
        [pins.gpio22.reconfigure().into_dyn_pin()];

    // Light one LED at a time. Start at GPIO2 and go through to GPIO5, then reverse.
    loop {
        for led in pinarray.iter_mut() {
            led.set_high().unwrap();
            timer.delay_ms(50);
            led.set_low().unwrap();
        }
        for led in pinarray.iter_mut().rev() {
            led.set_high().unwrap();
            timer.delay_ms(50);
            led.set_low().unwrap();
        }
    }
}

// End of file
