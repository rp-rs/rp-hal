//! # An example application with two Blocks.
//!
//! Our first block is an *Image Definition* Block and it points at our second
//! Block which contains a placeholder item for the purposes of this example.
//! The placeholder isn't useful per se, but it allows us to show how to
//! construct two Blocks in a loop without having to use `picotool` to modify
//! the application after compilation and linking.
//!
//! See [Section
//! 5.1.5.2](https://rptl.io/rp2350-datasheet#bootrom-concept-block-loop) in the
//! RP2350 datasheet for more information about Blocks and Block Loops.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

// Some things we need
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

// The linker script exports these symbols.
extern "C" {
    /// This value is at an address equal to the difference between the start block and end block
    static start_to_end: u32;
    /// This value is at an address equal to the difference between the end block and start block
    static end_to_start: u32;
}

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
#[allow(unused_unsafe)] // addr_of! is safe since rust 1.82.0
pub static START_IMAGE_DEF: hal::block::ImageDef =
    hal::block::ImageDef::secure_exe().with_offset(unsafe { core::ptr::addr_of!(start_to_end) });

/// A second Block, and the end of the program in flash
#[link_section = ".end_block"]
#[used]
#[allow(unused_unsafe)] // addr_of! is safe since rust 1.82.0
pub static END_IMAGE_DEF: hal::block::Block<1> =
    // Put a placeholder item in the block.
    hal::block::Block::new([hal::block::item_ignored()])
            .with_offset(unsafe { core::ptr::addr_of!(end_to_start) });

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

    // Configure GPIO25 as an output
    let mut led_pin = pins.gpio25.into_push_pull_output();
    loop {
        led_pin.set_high().unwrap();
        timer.delay_ms(250);
        led_pin.set_low().unwrap();
        timer.delay_ms(250);
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(
        c"Blinks an LED, contains a Block Loop with two Blocks"
    ),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
