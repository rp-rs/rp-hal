//! # SPI Bus example
//!
//! This application demonstrates how to construct a simple SPI Driver and
//! configure rp2040-hal's SPI peripheral to access it by utilising
//! [`ExclusiveDevice`] from [`embedded-hal-bus`].
//!
//! [`ExclusiveDevice`]:
//!     https://docs.rs/embedded-hal-bus/latest/embedded_hal_bus/spi/struct.ExclusiveDevice.html
//! [`embedded-hal-bus`]: https://crates.io/crates/embedded-hal-bus
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment.
//!
//! See the top-level `README.md` file for Copyright and license details.
//!
//! ## Glossary
//!
//! * *SPI Bus* - a shared bus consisting of three shared wires (Clock, COPI and
//!   CIPO) and a unique 'Chip Select' wire for each device on the bus. An *SPI
//!   Bus* typically only has one *SPI Controller* which is 'driving' the bus
//!   (specifically it is driving the clock signal and the *COPI* data line).
//! * *COPI* - One of the data lines on an *SPI Bus*. Stands for *Controller
//!   Out, Peripheral In*, but we use the term *SPI Device* instead of *SPI
//!   Peripheral*.
//! * *CIPO* - One of the data lines on an *SPI Bus*. Stands for *Controller In,
//!   Peripheral Out*, but we use the term *SPI Device* instead of *SPI
//!   Peripheral*.
//! * *SPI Controller* - a block of silicon within the RP2040 designed for
//!   driving an *SPI Bus*.
//! * *SPI Device* - a device on the *SPI Bus*; has its own unique chip select
//!   signal.
//! * *Device Driver* - a Rust type (and/or a value of that type) which
//!   represents a particular *SPI Device*, such as an Bosch BMA400
//!   accelerometer chip, or an ST7789 LCD controller chip.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// Some types/traits we need
use core::fmt::Write;
use embedded_hal::spi::Operation;
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use hal::clocks::Clock;
use hal::fugit::RateExtU32;
use hal::gpio::PinState;
use hal::uart::{DataBits, StopBits, UartConfig};

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

/// The ways our device driver might fail.
///
/// When things go wrong, we could return `Err(())`` but that wouldn't help
/// anyone troubleshoot so we'll create an error type with additional info to
/// return.
#[derive(Copy, Clone, Debug)]
pub enum MyError<E> {
    /// We got an error from the *SPI Device*
    Spi(E),
    // Add other errors for your driver here.
}

/// Our *Device Driver* type.
///
/// This is an example of a device driver that manages some kind of *SPI
/// Device*.
///
/// It is not a driver for any real device - it's here as an example of how to
/// write such a driver. You would typically get real drivers from a library
/// crate but we've pasted it into the example so you can see it.
///
/// We need to use a generic here (`SD`), because we want our example
/// driver to work for any other microcontroller that implements the
/// embedded-hal SPI traits - specifically the `embedded_hal::spi::SpiDevice`
/// trait that represents access to a unique device on the bus.
pub struct MySpiDeviceDriver<SD> {
    spi_dev: SD,
}

impl<SD> MySpiDeviceDriver<SD>
where
    SD: SpiDevice,
{
    /// Construct a new instance of our *Device Driver*.
    ///
    /// Takes ownership of a value representing a unique device on the *SPI
    /// Bus*.
    pub fn new(spi_dev: SD) -> Self {
        Self { spi_dev }
    }

    /// Write to our hypothetical device.
    ///
    /// We imagine our device has a register at `0x20`, that accepts a `u8`
    /// value.
    pub fn set_value(&mut self, value: u8) -> Result<(), MyError<SD::Error>> {
        self.spi_dev
            .transaction(&mut [Operation::Write(&[0x20, value])])
            .map_err(MyError::Spi)?;

        Ok(())
    }

    /// Read from our hypothetical device.
    ///
    /// We imagine our device has a register at `0x90`, that we can read a 2
    /// byte integer value from.
    pub fn get_value(&mut self) -> Result<u16, MyError<SD::Error>> {
        let mut buf = [0u8; 2];
        self.spi_dev
            .transaction(&mut [Operation::Write(&[0x90]), Operation::Read(&mut buf)])
            .map_err(MyError::Spi)?;

        Ok(u16::from_le_bytes(buf))
    }
}

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls
/// this function as soon as all global variables and the spinlock are
/// initialised.
///
/// The function configures the RP2040 peripherals, then performs some example
/// SPI transactions, then goes to sleep.
#[rp2040_hal::entry]
fn main() -> ! {
    // ========================================================================
    // Some things that pretty much every rp2040-hal program will do
    // ========================================================================

    // Grab our singleton objects
    let mut p = hal::pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(p.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        p.XOSC,
        p.CLOCKS,
        p.PLL_SYS,
        p.PLL_USB,
        &mut p.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(p.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

    // ========================================================================
    // Construct a timer object, which we will need later.
    // ========================================================================

    let timer = hal::Timer::new(p.TIMER, &mut p.RESETS, &clocks);

    // ========================================================================
    // Set up a UART so we can print out the values we read using our *Device
    // Driver*:
    // ========================================================================

    // How we want our UART to be configured
    let uart_config = UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One);

    // The pins we want to use for our UART.
    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_function(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_function(),
    );

    // Create new, uninitialized UART driver with one of the two UART objects
    // from our PAC, and our UART pins. We need temporary access to the RESETS
    // object to reset the UART.
    let uart = hal::uart::UartPeripheral::new(p.UART0, uart_pins, &mut p.RESETS);

    // Swap the uninitialised UART driver for an initialised one, passing in the
    // desired configuration. We need to know the internal UART clock frequency
    // to set up the baud rate dividers correctly.
    let mut uart = uart
        .enable(uart_config, clocks.peripheral_clock.freq())
        .unwrap();

    // ========================================================================
    // Set up our *SPI Bus* and one *SPI Device* on the bus, and then build our
    // *Device Driver*:
    // ========================================================================

    // Set up our SPI pins so they can be used by the *SPI Bus* driver.
    let spi_copi = pins.gpio7.into_function::<hal::gpio::FunctionSpi>();
    let spi_cipo = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio6.into_function::<hal::gpio::FunctionSpi>();

    // Create new, uninitialized *SPI Bus* driver with one of the two *SPI*
    // objects from our PAC, plus our data/clock pins for the *SPI Bus*.
    //
    // We've stubbed out most of the type parameters because the compiler can
    // work them out for us. The only one we need to specify is the size of a
    // word sent over the *SPI Bus* to our device - and we picked 8 bits (a
    // byte).
    let spi_bus = hal::spi::Spi::<_, _, _, 8>::new(p.SPI0, (spi_copi, spi_cipo, spi_sclk));

    // Exchange the uninitialised *SPI Bus* driver for an initialised one, by
    // passing in the extra bus parameters required.
    //
    // We need temporary access to the RESETS object to reset the SPI
    // peripheral, and we need to know the internal clock speed in order to set
    // up the clock dividers correctly.
    let spi_bus = spi_bus.init(
        &mut p.RESETS,
        clocks.peripheral_clock.freq(),
        16.MHz(),
        embedded_hal::spi::MODE_0,
    );

    // The Chip Select pin. Every *SPI Device* has a unique chip select signal,
    // and when this pin goes low, it uniquely selects our desired
    // (hypothetical) *SPI Device* on the *SPI Bus*.
    let spi_cs = pins.gpio8.into_push_pull_output_in_state(PinState::High);

    // Create an object which represents our (hypothetical) *SPI Device* on the
    // *SPI Bus*.
    //
    // We are the only task talking to this *SPI Bus*, so we can use
    // `ExclusiveDevice` here. If we had multiple tasks accessing the same bus
    // (e.g. you had both an accelerometer and a pressure sensor on the same
    // bus), we would need to use a different interface to ensure that we have
    // exclusive access to this bus (via `AtomicDevice` or
    // `CriticalSectionDevice`, for example).
    //
    // See https://docs.rs/embedded-hal-bus/0.2.0/embedded_hal_bus/spi for a
    // list of options.
    //
    // We can safely unwrap here, because the only possible failure is CS
    // assertion failure and our CS pin is infallible.
    //
    // Note that it also takes ownership of our timer object so that it has a
    // way of measuring elapsed time. This is required so it can handle `Delay`
    // transactions that can put small pauses inbetween say, a `Write`
    // transaction and a `Read` transaction (which some SPI devices require
    // because they're a bit sluggish and take time to process things).
    let excl_spi_dev = ExclusiveDevice::new(spi_bus, spi_cs, timer).unwrap();

    // Now that we've constructed a value of type `ExclusiveDevice` (which
    // implements the embedded-hal `SpiDevice` traits) we can finally construct
    // our device driver.
    let mut driver = MySpiDeviceDriver::new(excl_spi_dev);

    // ========================================================================
    // Now let's use our device driver.
    // ========================================================================

    // Let's write to the device
    match driver.set_value(10) {
        Ok(_) => {
            // Do something on success
            _ = writeln!(uart, "Wrote to device OK!");
        }
        Err(e) => {
            // Do something on failure
            _ = writeln!(uart, "Device write error: {:?}", e);
        }
    }

    // Let's read from the device
    match driver.get_value() {
        Ok(value) => {
            // Do something on success
            _ = writeln!(uart, "Read value: {}", value);
        }
        Err(e) => {
            // Do something on failure
            _ = writeln!(uart, "Device write error: {:?}", e);
        }
    }

    // We're done, so just sleep in an infinite loop. Your program should
    // probably do something more useful.
    loop {
        cortex_m::asm::wfi();
    }
}

// End of file
