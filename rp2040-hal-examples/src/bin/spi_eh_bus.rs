//! # SPI Bus example
//!
//! This application demonstrates how to construct a simple Spi Driver,
//! and configure rp2040-hal's Spi peripheral to access it by utilising
//! `ExclusiveDevice`` from `embedded_hal_bus`
//!
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment.
//!
//! See the top-level `README.md` file for Copyright and license details.

#![no_std]
#![no_main]

use embedded_hal::spi::Operation;
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use rp2040_hal::gpio::PinState;
use rp2040_hal::uart::{DataBits, StopBits, UartConfig};
// Alias for our HAL crate
use rp2040_hal as hal;

// Some traits we need
use core::fmt::Write;
use hal::clocks::Clock;
use hal::fugit::RateExtU32;

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

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

/// Our Spi device driver
/// We need to use a generic here (SPI), because we want our driver to work for any other
/// microcontroller that implements the embedded-hal Spi traits
struct MySpiDriver<SPI> {
    spi: SPI,
}

/// When things go wrong, we could return Error(()) but that wouldn't help anyone troubleshoot
/// so we'll create an error type with additional info to return.
#[derive(Copy, Clone, Debug)]
enum MyError<SPI> {
    Spi(SPI),
    // Add other errors for your driver here.
}

/// Implementation of the business logic for the remote SPI IC
impl<SPI> MySpiDriver<SPI>
where
    SPI: SpiDevice,
{
    /// Construct a new instance of SPI device driver
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Our hypothetical SPI device has a register at 0x20, that accepts a u8 value
    pub fn set_value(&mut self, value: u8) -> Result<(), MyError<SPI::Error>> {
        self.spi
            .transaction(&mut [Operation::Write(&[0x20, value])])
            .map_err(MyError::Spi)?;

        Ok(())
    }

    /// Our hypothetical Spi device has a register at 0x90, that we can read a 2 byte value from
    pub fn get_value(&mut self) -> Result<[u8; 2], MyError<SPI::Error>> {
        let mut buf = [0; 2];
        self.spi
            .transaction(&mut [Operation::Write(&[0x90]), Operation::Read(&mut buf)])
            .map_err(MyError::Spi)?;

        Ok(buf)
    }
}

/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then performs some example
/// SPI transactions, then goes to sleep.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
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

    let timer = rp2040_hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio0.into_function(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio1.into_function(),
    );

    // Set up a uart so we can print out the values from our SPI peripheral
    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    // Set up our SPI pins so they can be used by the SPI driver
    let spi_mosi = pins.gpio7.into_function::<hal::gpio::FunctionSpi>();
    let spi_miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
    let spi_sclk = pins.gpio6.into_function::<hal::gpio::FunctionSpi>();

    // Chip select is handled by SpiDevice, not SpiBus. It logically belongs with the specific SPI device we're talking to
    let spi_cs = pins.gpio8.into_push_pull_output_in_state(PinState::High);

    // Create new, uninitialized SPI bus with one of the 2 SPI peripherals from our PAC, and our SPI data/clock pins
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));

    // Exchange the uninitialised Spi bus for an initialised one, passing in the extra bus parameters required
    let spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16.MHz(),
        embedded_hal::spi::MODE_0,
    );

    // We are the only task talking to this SPI peripheral, so we can use ExclusiveDevice here.
    // If we had multiple tasks accessing this, you would need to use a different interface to
    // ensure that we have exclusive access to this bus (via AtomicDevice or CriticalSectionDevice, for example)
    //
    // We can safely unwrap here, because the only possible failure is CS assertion failure, but our CS pin is infallible
    let excl_dev = ExclusiveDevice::new(spi, spi_cs, timer).unwrap();

    // Now that we've constructed a SpiDevice for our driver to use, we can finally construct our Spi driver
    let mut driver = MySpiDriver::new(excl_dev);

    match driver.set_value(10) {
        Ok(_) => {
            // Do something on success
        }
        Err(_) => {
            // Do something on failure
        }
    }

    match driver.get_value() {
        Ok(value) => {
            // Do something on success
            writeln!(uart, "Read value was {} {}\n", value[0], value[1]).unwrap();
        }
        Err(_) => {
            // Do something on failure
        }
    }

    loop {
        cortex_m::asm::wfi();
    }
}

// End of file
