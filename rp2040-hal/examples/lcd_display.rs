//! # LCD Display Example
//!
//! In this example, the RP2040 is configured to drive a small two-line
//! alphanumeric LCD using the
//! [HD44780](https://crates.io/crates/hd44780-driver) driver.
//!
//! It drives the LCD by pushing data out of six GPIO pins. It may need to be
//! adapted to your particular board layout and/or pin assignment.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The macro for our start-up function
use cortex_m_rt::entry;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// Our LCD driver
use hd44780_driver as hd44780;

// Some traits we need
use embedded_time::fixed_point::FixedPoint;
use rp2040_hal::clocks::Clock;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, writes to the LCD, then goes
/// to sleep.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

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

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Create the LCD driver from some GPIO pins
    let mut lcd = hd44780::HD44780::new_4bit(
        pins.gpio16.into_push_pull_output(), // Register Select
        pins.gpio17.into_push_pull_output(), // Enable
        pins.gpio18.into_push_pull_output(), // d4
        pins.gpio19.into_push_pull_output(), // d5
        pins.gpio20.into_push_pull_output(), // d6
        pins.gpio21.into_push_pull_output(), // d7
        &mut delay,
    )
    .unwrap();

    // Clear the screen
    lcd.reset(&mut delay).unwrap();
    lcd.clear(&mut delay).unwrap();

    // Write to the top line
    lcd.write_str("rp-hal on", &mut delay).unwrap();

    // Move the cursor
    lcd.set_cursor_pos(40, &mut delay).unwrap();

    // Write more more text
    lcd.write_str("HD44780!", &mut delay).unwrap();

    // Do nothing - we're finished
    #[allow(clippy::empty_loop)]
    loop {
        // Empty loop
    }
}

// End of file
