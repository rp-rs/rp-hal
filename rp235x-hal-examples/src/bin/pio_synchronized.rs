//! This example toggles the GPIO0 and GPIO1 pins, with each controlled from a
//! separate PIO state machine.
//!
//! Despite running in separate state machines, the clocks are synchronized at
//! the rise and fall times will be simultaneous.
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

    // configure pins for Pio0.
    let gp0: Pin<_, FunctionPio0, _> = pins.gpio0.into_function();
    let gp1: Pin<_, FunctionPio0, _> = pins.gpio1.into_function();

    // PIN id for use inside of PIO
    let pin0 = gp0.id().num;
    let pin1 = gp1.id().num;

    // Define some simple PIO program.
    let program = pio_proc::pio_asm!(
        "
.wrap_target
    set pins, 1 [31]
    set pins, 0 [31]
.wrap
        "
    );

    // Initialize and start PIO
    let (mut pio, sm0, sm1, _, _) = pac.PIO0.split(&mut pac.RESETS);
    // I'm "measuring" the phase offset between the two pins by connecting
    // then through a LED. If there is a clock offset, there will be a
    // short time with a voltage between the pins, so the LED will flash up.
    // With a slow clock this is not visible, so use a reasonably fast clock.
    let (int, frac) = (256, 0);

    let installed = pio.install(&program.program).unwrap();
    let (mut sm0, _, _) = hal::pio::PIOBuilder::from_installed_program(
        // Safety: We won't uninstall the program, ever
        unsafe { installed.share() },
    )
    .set_pins(pin0, 1)
    .clock_divisor_fixed_point(int, frac)
    .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm0.set_pindirs([(pin0, hal::pio::PinDir::Output)]);

    let (mut sm1, _, _) = hal::pio::PIOBuilder::from_installed_program(installed)
        .set_pins(pin1, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm1);
    // The GPIO pin needs to be configured as an output.
    sm1.set_pindirs([(pin1, hal::pio::PinDir::Output)]);

    // Start both SMs at the same time
    let group = sm0.with(sm1).sync().start();
    hal::arch::delay(10_000_000);

    // Stop both SMs at the same time
    let group = group.stop();
    hal::arch::delay(10_000_000);

    // Start them again and extract the individual state machines
    let (sm1, sm2) = group.start().free();
    hal::arch::delay(10_000_000);

    // Stop the two state machines separately
    let _sm1 = sm1.stop();
    hal::arch::delay(10_000_000);
    let _sm2 = sm2.stop();

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
    hal::binary_info::rp_program_description!(c"PIO Synchronisation Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
