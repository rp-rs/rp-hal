//! General Purpose Input and Output (GPIO)
//!
//! See [`pin`](self::pin) for implementation details and in-depth documentation.
//!
//! ## Basic usage
//! ```no_run
//! use embedded_hal::digital::v2::{InputPin, OutputPin};
//! use rp2040_hal::{clocks::init_clocks_and_plls, gpio::Pins, watchdog::Watchdog, pac, Sio};
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
//! let mut clocks = init_clocks_and_plls(XOSC_CRYSTAL_FREQ, peripherals.XOSC, peripherals.CLOCKS, peripherals.PLL_SYS, peripherals.PLL_USB, &mut peripherals.RESETS, &mut watchdog).ok().unwrap();
//!
//! let mut pac = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(pac.SIO);
//! let pins = rp2040_hal::gpio::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
//! // Set a pin to drive output
//! let mut output_pin = pins.gpio25.into_push_pull_output();
//! // Drive output to 3.3V
//! output_pin.set_high().unwrap();
//! // Drive output to 0V
//! output_pin.set_low().unwrap();
//! // Set a pin to input
//! let input_pin = pins.gpio24.into_floating_input();
//! // pinstate will be true if the pin is above 2V
//! let pinstate = input_pin.is_high().unwrap();
//! // pinstate_low will be true if the pin is below 1.15V
//! let pinstate_low = input_pin.is_low().unwrap();
//! // you'll want to pull-up or pull-down a switch if it's not done externally
//! let button_pin = pins.gpio23.into_pull_down_input();
//! let button2_pin = pins.gpio22.into_pull_up_input();
//! ```
//! See [examples/gpio_in_out.rs](https://github.com/rp-rs/rp-hal/tree/main/rp2040-hal/examples/gpio_in_out.rs) for a more practical example

// Based heavily on and in some places copied from `atsamd-hal` gpio::v2
pub mod pin;
pub use pin::*;

pub mod dynpin;
pub use dynpin::*;

mod reg;

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// The amount of current that a pin can drive when used as an output
pub enum OutputDriveStrength {
    /// 2 mA
    TwoMilliAmps,
    /// 4 mA
    FourMilliAmps,
    /// 8 mA
    EightMilliAmps,
    /// 12 mA
    TwelveMilliAmps,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// The slew rate of a pin when used as an output
pub enum OutputSlewRate {
    /// Slew slow
    Slow,
    /// Slew fast
    Fast,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// Interrupt kind
pub enum Interrupt {
    /// While low
    LevelLow,
    /// While high
    LevelHigh,
    /// On falling edge
    EdgeLow,
    /// On rising edge
    EdgeHigh,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// Interrupt override state.
pub enum InterruptOverride {
    /// Don't invert the interrupt.
    DontInvert = 0,
    /// Invert the interrupt.
    Invert = 1,
    /// Drive interrupt low.
    AlwaysLow = 2,
    /// Drive interrupt high.
    AlwaysHigh = 3,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// Input override state.
pub enum InputOverride {
    /// Don't invert the peripheral input.
    DontInvert = 0,
    /// Invert the peripheral input.
    Invert = 1,
    /// Drive peripheral input low.
    AlwaysLow = 2,
    /// Drive peripheral input high.
    AlwaysHigh = 3,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// Output enable override state.
pub enum OutputEnableOverride {
    /// Use the original output enable signal from selected peripheral.
    DontInvert = 0,
    /// Invert the output enable signal from selected peripheral.
    Invert = 1,
    /// Disable output.
    Disable = 2,
    /// Enable output.
    Enable = 3,
}

#[derive(Clone, Copy, Eq, PartialEq, Debug)]
/// Output override state.
pub enum OutputOverride {
    /// Use the original output signal from selected peripheral.
    DontInvert = 0,
    /// Invert the output signal from selected peripheral.
    Invert = 1,
    /// Drive output low.
    AlwaysLow = 2,
    /// Drive output high.
    AlwaysHigh = 3,
}
