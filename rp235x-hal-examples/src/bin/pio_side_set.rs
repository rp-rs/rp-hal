//! This example toggles the GPIO25 pin, using a PIO program compiled via pio_proc::pio!().
//!
//! If a LED is connected to that pin, like on a Pico board, the LED should blink.
//!
//! This example makes use of side setting.
#![no_std]
#![no_main]

use rp235x_hal as hal;

use hal::gpio::{FunctionPio0, Pin};
use hal::pio::PIOExt;
use hal::Sio;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
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
    let program = pio_proc::pio_asm!(
        ".side_set 1", // each instruction must set 1 bit
        ".wrap_target",
        "    nop side 1 [15]",
        "    nop side 0 [15]",
        ".wrap",
    );

    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program.program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
    let (mut sm, _, _) = hal::pio::PIOBuilder::from_installed_program(installed)
        .side_set_pin_base(led_pin_id)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm.set_pindirs([(led_pin_id, hal::pio::PinDir::Output)]);
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
    hal::binary_info::rp_program_description!(c"PIO Side-set Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
