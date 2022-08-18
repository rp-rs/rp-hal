//! # LCD Display Example
//!
//! In this example, the RP2040 is configured to drive a small two-line
//! alphanumeric LCD using the
//! [HD44780](https://crates.io/crates/hd44780-driver) driver.
//!
//! This example drives the LCD by pushing data out of six GPIO pins, writing
//! the data four bits at a time. A faster alternative can be created using
//! HD44780::new_8bit() but requiring an additional four GPIO pins.
//!
//! See the `Cargo.toml` file for Copyright and license details.
//!
//! ```text
//!                      /--------------------------------------\
//!  ____________        |        /-------------------------\   |
//! | 1       GND|-------+---\    |           _|USB|_       |   |
//! | 2       VDD|-------+---+----/          |1  R 40|-VBUS-o   v
//! | 3        VS|-------/   |               |2  P 39|       ||POT||
//! | 4        RS|--\        o-----------GND-|3    38|-GND----------o
//! | 5        RW|--+--------/    /------GP2-|4  P 37|
//! | 6        EN|--+-\        /--+------GP3-|5  I 36|
//! | 7          |  | |     /--+--+------GP4-|6  C   |
//! | 8          |  | |  /--+--+--+------GP5-|7  O   |
//! | 9          |  | \--+--+--+--+---\      |8      |
//! | 10         |  \----+--+--+--+-\  \-GP6-|9      |
//! | 11       D4|-------/  |  |  |  \---GP7-|10     |
//! | 12       D5|----------/  |  |          .........
//! | 13       D6|-------------/  |          |20   21|
//! | 14       D7|----------------/           """""""
//! ..............
//! Symbols:
//!     - (+) crossing lines, not connected
//!     - (o) connected lines
//! ```
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// GPIO traits
use embedded_hal::digital::v2::OutputPin;

// For LCD display
use hd44780_driver::HD44780;

/// Entry point to our bare-metal application.
///
/// The `#[rp_pico::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
#[rp_pico::entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = rp_pico::hal::pac::Peripherals::take().unwrap();
    let core = rp_pico::hal::pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = rp_pico::hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    // The default is to generate a 125 MHz system clock
    let clocks = rp_pico::hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
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
    let sio = rp_pico::hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut led_pin = pins.led.into_push_pull_output();

    // The delay object lets us wait for specified amounts of time
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Init pins
    let rs = pins.gpio7.into_push_pull_output();
    let en = pins.gpio6.into_push_pull_output();
    let d4 = pins.gpio5.into_push_pull_output();
    let d5 = pins.gpio4.into_push_pull_output();
    let d6 = pins.gpio3.into_push_pull_output();
    let d7 = pins.gpio2.into_push_pull_output();

    // LCD Init
    let mut lcd = HD44780::new_4bit(rs, en, d4, d5, d6, d7, &mut delay).unwrap();

    loop {
        // Clear the screen
        lcd.reset(&mut delay).unwrap();
        lcd.clear(&mut delay).unwrap();

        // Write to the top line
        lcd.write_str("rp-hal on", &mut delay).unwrap();

        // Move the cursor
        lcd.set_cursor_pos(40, &mut delay).unwrap();

        // Write more more text
        lcd.write_str("HD44780! ", &mut delay).unwrap();
        let mut char_count = 9;
        for ch in "move along!.. ".chars() {
            if char_count > 15 {
                // Switch autoscroll on
                lcd.set_autoscroll(true, &mut delay).unwrap();
            }
            led_pin.set_high().unwrap();
            lcd.write_char(ch, &mut delay).unwrap();
            char_count += 1;
            delay.delay_us(400_000); //0.4s
            led_pin.set_low().unwrap();
            delay.delay_us(100_000); //0.1s
        }
        lcd.set_autoscroll(false, &mut delay).unwrap();
    }
}
