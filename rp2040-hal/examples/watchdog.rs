//! # Watchdog Example
//!
//! This application demonstrates how to use the RP2040 Watchdog.
//!
//! It may need to be adapted to your particular board layout and/or pin assignment.
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

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;

// Some traits we need
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
use embedded_time::duration::Extensions;
use embedded_time::fixed_point::FixedPoint;
use rp2040_hal::clocks::Clock;

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
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. After a period of time, the watchdog will kick in to reset
/// the CPU.
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

    // Configure an LED so we can show the current state of the watchdog
    let mut led_pin = pins.gpio25.into_push_pull_output();

    // Set the LED high for 2 seconds so we know when we're about to start the watchdog
    led_pin.set_high().unwrap();
    delay.delay_ms(2000);

    // Set to watchdog to reset if it's not reloaded within 1.05 seconds, and start it
    watchdog.start(1_050_000u32.microseconds());

    // Blink once a second for 5 seconds, refreshing the watchdog timer once a second to avoid a reset
    for _ in 1..=5 {
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        watchdog.feed();
    }

    // Blink 10 times per second, not feeding the watchdog.
    // The processor should reset in 1.05 seconds, or 5 blinks time
    loop {
        led_pin.set_low().unwrap();
        delay.delay_ms(100);
        led_pin.set_high().unwrap();
        delay.delay_ms(100);
    }
}

// End of file
