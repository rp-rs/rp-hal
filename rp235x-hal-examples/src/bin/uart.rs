//! # UART Example
//!
//! This application demonstrates how to use the UART Driver to talk to a serial
//! connection.
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp235x_hal as hal;

use hal::gpio;

// Some things we need
use core::fmt::Write;
use embedded_hal::delay::DelayNs;
use hal::clocks::Clock;
use hal::fugit::RateExtU32;

// UART related types
use hal::uart::{DataBits, StopBits, UartConfig, ValidatedPinRx, ValidatedPinTx};

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// External high-speed crystal on the Raspberry Pi Pico 2 board is 12 MHz.
/// Adjust if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Entry point to our bare-metal application.
///
/// The `#[hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the rp235x peripherals, then writes to the UART in
/// an infinite loop.
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

    let mut delay = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart0_pins = (
        // UART TX (characters sent from rp235x) on pin 4 (GPIO2) in Aux mode
        pins.gpio2.into_function(),
        // UART RX (characters received by rp235x) on pin 5 (GPIO3) in Aux mode
        pins.gpio3.into_function(),
    );
    let mut uart0 = hal::uart::UartPeripheral::new(pac.UART0, uart0_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // Set UART1 up with some dynamic pins

    // UART TX (characters sent from rp235x) on pin 6 (GPIO4)
    //
    // We erase the type-state and make a dynamically typed pin. this is useful
    // if you want to store it in a struct, or pass it as an argument to a
    // library.
    let mut uart1_tx = pins
        .gpio4
        .reconfigure::<gpio::DynFunction, gpio::DynPullType>()
        .into_dyn_pin();
    // try and put it into UART mode (the type of the variable doesn't change)
    if uart1_tx.try_set_function(gpio::DynFunction::Uart).is_err() {
        panic!("Can't set pin as UART")
    }
    // wrap it, to prove to the UartPeripheral that it *is* in Uart mode
    let Ok(uart1_tx) = ValidatedPinTx::validate(uart1_tx, &pac.UART1) else {
        panic!("Can't use pin for UART 1 TX")
    };

    // UART RX (characters received by rp235x) on pin 7 (GPIO5)
    //
    // We erase the type-state and make a dynamically typed pin. this is useful
    // if you want to store it in a struct, or pass it as an argument to a
    // library.
    let mut uart1_rx = pins
        .gpio5
        .reconfigure::<gpio::DynFunction, gpio::DynPullType>()
        .into_dyn_pin();
    // try and put it into UART mode
    if uart1_rx.try_set_function(gpio::DynFunction::Uart).is_err() {
        panic!("Can't set pin as UART")
    }
    // wrap it, to prove to the UartPeripheral that it *is* in Uart mode
    let Ok(uart1_rx) = ValidatedPinRx::validate(uart1_rx, &pac.UART1) else {
        panic!("Can't use pin for UART 1 RX")
    };
    // make a UART with our dynamic pin types
    let uart1_pins = (uart1_tx, uart1_rx);
    let mut uart1 = hal::uart::UartPeripheral::new(pac.UART1, uart1_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    uart0.write_full_blocking(b"UART example on UART0\r\n");
    uart1.write_full_blocking(b"UART example on UART1\r\n");

    let mut value = 0u32;
    loop {
        writeln!(uart0, "UART0 says value: {value:02}\r").unwrap();
        writeln!(uart1, "UART1 says value: {value:02}\r").unwrap();
        delay.delay_ms(1000);
        value += 1
    }
}

/// Program metadata for `picotool info`
#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [hal::binary_info::EntryAddr; 5] = [
    hal::binary_info::rp_cargo_bin_name!(),
    hal::binary_info::rp_cargo_version!(),
    hal::binary_info::rp_program_description!(c"UART Example"),
    hal::binary_info::rp_cargo_homepage_url!(),
    hal::binary_info::rp_program_build_attribute!(),
];

// End of file
