//! # GPIO Dynamic Pin Type Mode Example
//!
//! This application demonstrates how to put GPIO pins into their Dynamic Pin type on the rp235x.
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

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

use hal::gpio::{DynPinId, FunctionSioOutput, Pin, PullNone, PullUp};

// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
#[hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = hal::pac::Peripherals::take().unwrap();

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
    .unwrap();

    // We will use the rp235x timer peripheral as our delay source
    let mut timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

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

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"GPIO Dynamic Pin Array Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
