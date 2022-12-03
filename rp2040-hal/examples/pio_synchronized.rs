//! This example toggles the GPIO0 and GPIO1 pins, with each controlled from a
//! separate PIO state machine.
//!
//! Despite running in separate state machines, the clocks are sychronized at
//! the rise and fall times will be simultaneous.
#![no_std]
#![no_main]

use hal::gpio::{FunctionPio0, Pin};
use hal::pac;
use hal::pio::PIOExt;
use hal::Sio;
use panic_halt as _;
use rp2040_hal as hal;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // configure pins for Pio0.
    let _: Pin<_, FunctionPio0> = pins.gpio0.into_mode();
    let _: Pin<_, FunctionPio0> = pins.gpio1.into_mode();

    // PIN id for use inside of PIO
    let pin0 = 0;
    let pin1 = 1;

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
    let (mut sm0, _, _) = rp2040_hal::pio::PIOBuilder::from_program(
        // Safety: We won't uninstall the program, ever
        unsafe { installed.share() },
    )
    .set_pins(pin0, 1)
    .clock_divisor_fixed_point(int, frac)
    .build(sm0);
    // The GPIO pin needs to be configured as an output.
    sm0.set_pindirs([(pin0, hal::pio::PinDir::Output)]);

    let (mut sm1, _, _) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .set_pins(pin1, 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm1);
    // The GPIO pin needs to be configured as an output.
    sm1.set_pindirs([(pin1, hal::pio::PinDir::Output)]);

    // Start both SMs at the same time
    let group = sm0.with(sm1).sync().start();
    cortex_m::asm::delay(10_000_000);

    // Stop both SMs at the same time
    let group = group.stop();
    cortex_m::asm::delay(10_000_000);

    // Start them again and extract the individual state machines
    let (sm1, sm2) = group.start().free();
    cortex_m::asm::delay(10_000_000);

    // Stop the two state machines separately
    let _sm1 = sm1.stop();
    cortex_m::asm::delay(10_000_000);
    let _sm2 = sm2.stop();

    loop {
        cortex_m::asm::wfi();
    }
}
