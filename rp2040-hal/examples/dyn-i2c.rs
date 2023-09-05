//! # I²C Example
//!
//! This application demonstrates how to talk to I²C devices with an RP2040.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Some traits we need
use embedded_hal::blocking::i2c::Write;
use fugit::RateExtU32;

// Alias for our HAL crate
use rp2040_hal as hal;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::{gpio::new_pin, pac};

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
/// The function configures the RP2040 peripherals, then performs a single I²C
/// write to a fixed address.
#[rp2040_hal::entry]
fn main() -> ! {
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Create the I²C drive. The pins are acquired without taking
    // ownership from `pins` - the developer is in charge of ensuring
    // that nothing else uses I2C0 and the pins.
    let mut i2c = hal::I2C::new_controller(
        unsafe { pac::Peripherals::steal().I2C0 },
        unsafe {
            new_pin(pins.gpio18.id())
                .try_into_function()
                .unwrap_or_else(|_| panic!("gpio18 couldn't be allocated"))
        },
        unsafe {
            new_pin(pins.gpio19.id())
                .try_into_function()
                .unwrap_or_else(|_| panic!("gpio19 couldn't be allocated"))
        },
        400.kHz().into(),
        &mut pac.RESETS,
        (&clocks.system_clock).into(),
    );

    // Write three bytes to the I²C device with 7-bit address 0x2C
    i2c.write(0x2c, &[1, 2, 3]).unwrap();

    // Demo finish - just loop until reset

    loop {
        cortex_m::asm::wfi();
    }
}

// End of file
