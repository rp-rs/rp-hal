//! # GPIO 'Blinky' Example
//!
//! This application demonstrates how to control a GPIO pin on the RP2040.
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
use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::fixed_point::FixedPoint;
use hal::gpio::dynpin::DynPin;
use hal::Clock;

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

use dht11::Dht11;

struct InOutPin {
    inner: DynPin,
}

impl InputPin for InOutPin {
    type Error = rp2040_hal::gpio::Error;
    fn is_high(&self) -> Result<bool, <Self as embedded_hal::digital::v2::InputPin>::Error> {
        self.inner.is_high()
    }
    fn is_low(&self) -> Result<bool, <Self as embedded_hal::digital::v2::InputPin>::Error> {
        self.inner.is_low()
    }
}

impl OutputPin for InOutPin {
    type Error = rp2040_hal::gpio::Error;
    fn set_low(&mut self) -> Result<(), <Self as embedded_hal::digital::v2::OutputPin>::Error> {
        self.inner.into_readable_output();
        self.inner.set_low()
    }
    fn set_high(&mut self) -> Result<(), <Self as embedded_hal::digital::v2::OutputPin>::Error> {
        self.inner.into_pull_up_input();
        Ok(())
    }
}

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then toggles a GPIO pin in
/// an infinite loop. If there is an LED connected to that pin, it will blink.
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    // Configure GPIO 25 as an output
    let pin = pins.gpio0.into_push_pull_output();
    let pin = InOutPin { inner: pin.into() };

    // Create an instance of the DHT11 device
    let mut dht11 = Dht11::new(pin);

    // Perform a sensor reading
    let _measurement = dht11.perform_measurement(&mut delay).unwrap();

    #[allow(clippy::empty_loop)]
    loop {
        // Empty loop
    }
}

// End of file
