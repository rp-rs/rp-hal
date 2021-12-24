//! # Pico I2C PIO Example
//!
//! Reads the temperature from an LM75B
//!
//! This read over I2C the temerature from an LM75B temperature sensor wired on pins 20 and 21
//! using the PIO peripheral as an I2C bus controller.
//! The pins used for the I2C can be remapped to any other pin available to the PIO0 peripheral.
//!
//! See the `Cargo.toml` file for Copyright and licence details.

#![no_std]
#![no_main]

// The trait used by formatting macros like write! and writeln!
use core::fmt::Write as FmtWrite;

// The macro for our start-up function
use cortex_m_rt::entry;

// I2C HAL traits & Types.
use embedded_hal::blocking::i2c::{Operation, Read, Transactional, Write};

// Time handling traits
use embedded_time::rate::*;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

/// Prints the temperature received from the sensor
fn print_temperature(serial: &mut impl FmtWrite, temp: [u8; 2]) {
    let temp_i16 = i16::from_be_bytes(temp) >> 5;
    let temp_f32 = f32::from(temp_i16) * 0.125;

    // Write formatted output but ignore any error.
    let _ = writeln!(serial, "Temperature: {:0.2}°C", temp_f32);
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, reads the temperature from
/// the attached LM75B using PIO0.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
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
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut uart = hal::uart::UartPeripheral::<_, _>::new(pac.UART0, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_115200_8_N_1,
            clocks.peripheral_clock.into(),
        )
        .unwrap();

    // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
    let _tx_pin = pins.gpio0.into_mode::<hal::gpio::FunctionUart>();
    // UART RX (characters reveived by RP2040) on pin 2 (GPIO1)
    let _rx_pin = pins.gpio1.into_mode::<hal::gpio::FunctionUart>();

    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let mut i2c_pio = i2c_pio::I2C::new(
        &mut pio,
        pins.gpio20,
        pins.gpio21,
        sm0,
        100_000.Hz(),
        clocks.system_clock.freq(),
    );

    let mut temp = [0; 2];
    i2c_pio
        .read(0x48u8, &mut temp)
        .expect("Failed to read from the peripheral");
    print_temperature(&mut uart, temp);

    i2c_pio
        .write(0x48u8, &[0])
        .expect("Failed to write to the peripheral");

    let mut temp = [0; 2];
    i2c_pio
        .read(0x48u8, &mut temp)
        .expect("Failed to read from the peripheral");
    print_temperature(&mut uart, temp);

    let mut config = [0];
    let mut thyst = [0; 2];
    let mut tos = [0; 2];
    let mut temp = [0; 2];
    let mut operations = [
        Operation::Write(&[1]),
        Operation::Read(&mut config),
        Operation::Write(&[2]),
        Operation::Read(&mut thyst),
        Operation::Write(&[3]),
        Operation::Read(&mut tos),
        Operation::Write(&[0]),
        Operation::Read(&mut temp),
    ];
    i2c_pio
        .exec(0x48u8, &mut operations)
        .expect("Failed to run all operations");
    print_temperature(&mut uart, temp);

    loop {
        cortex_m::asm::nop();
    }
}

// End of file
