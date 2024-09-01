//! This example toggles the GPIO25 pin, using a PIO program.
//!
//! If a LED is connected to that pin, like on a Pico board, the LED should blink.
#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use rp235x_hal as hal;

use hal::gpio::{FunctionPio0, Pin};
use hal::pio::PIOExt;
use hal::Sio;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then blinks an LED using the PIO peripheral.
#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // configure LED pin for Pio0.
    let led: Pin<_, FunctionPio0, _> = pins.gpio25.into_function();
    // PIN id for use inside of PIO
    let led_pin_id = led.id().num;

    // Define some simple PIO program.
    const MAX_DELAY: u8 = 31;
    let mut a = pio::Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    // Set pin as Out
    a.set(pio::SetDestination::PINDIRS, 1);
    // Define begin of program loop
    a.bind(&mut wrap_target);
    // Set pin low
    a.set_with_delay(pio::SetDestination::PINS, 0, MAX_DELAY);
    // Set pin high
    a.set_with_delay(pio::SetDestination::PINS, 1, MAX_DELAY);
    // Define end of program loop
    a.bind(&mut wrap_source);
    // The labels wrap_target and wrap_source, as set above,
    // define a loop which is executed repeatedly by the PIO
    // state machine.
    let program = a.assemble_with_wrap(wrap_source, wrap_target);

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
    let (sm, _, _) = hal::pio::PIOBuilder::from_installed_program(installed)
        .set_pins(led_pin_id, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    sm.start();

    // PIO runs in background, independently from CPU
    loop {
        hal::arch::wfi();
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"PIO Blinky Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
