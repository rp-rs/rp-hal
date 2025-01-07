//! # gpin External Clocks example
//!
//! This application demonstrates how to clock the processor using an external clock on GPIO20
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the top-level `README.md` file for Copyright and license details.

#![no_std]
#![no_main]

use embedded_hal_0_2::digital::v2::ToggleableOutputPin;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// To use the .MHz() function
use fugit::RateExtU32;

use rp2040_hal::clocks::ClockSource;
// Alias for our HAL crate
use rp2040_hal as hal;

// Necessary HAL types
use hal::{clocks::ClocksManager, gpin::GpIn0, gpio, Clock, Sio};

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

// The external clock provided to GPIO pin 20.
const GPIN_EXTERNAL_CLOCK_FREQ_HZ: u32 = 1_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 to accept an external clock on Gpio20,
/// then configures the system clock to run off this clock.
#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    let sio = Sio::new(pac.SIO);

    let pins = gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let gpin0_pin = pins.gpio20.reconfigure();
    let gpin0: GpIn0 = GpIn0::new(gpin0_pin, GPIN_EXTERNAL_CLOCK_FREQ_HZ.Hz());

    let mut clocks = ClocksManager::new(pac.CLOCKS);

    clocks
        .system_clock
        .configure_clock(&gpin0, gpin0.get_freq())
        .unwrap();

    let mut test_pin = pins.gpio0.into_push_pull_output();

    loop {
        // Continuously toggle a pin so it's possible to observe on a scope that the pico runs on
        // the externally provided frequency, and is synchronized to it.
        test_pin.toggle().unwrap();
    }
}
