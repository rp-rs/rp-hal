use crate::pac::{uart0::RegisterBlock, UART0, UART1};
use crate::resets::SubsystemReset;
use core::ops::Deref;
use embedded_time::rate::Baud;

/// Error type for UART operations.
#[derive(Debug)]
pub enum Error {
    /// Bad argument : when things overflow, ...
    BadArgument,
}
/// State of the UART Peripheral.
pub trait State {}

/// Trait to handle both underlying devices (UART0 & UART1)
pub trait UartDevice: Deref<Target = RegisterBlock> + SubsystemReset + 'static {}

impl UartDevice for UART0 {}
impl UartDevice for UART1 {}

/// UART is enabled.
pub struct Enabled;

/// UART is disabled.
pub struct Disabled;

impl State for Enabled {}
impl State for Disabled {}

/// Data bits
pub enum DataBits {
    /// 5 bits
    Five,
    /// 6 bits
    Six,
    /// 7 bits
    Seven,
    /// 8 bits
    Eight,
}

/// Stop bits
pub enum StopBits {
    /// 1 bit
    One,

    /// 2 bits
    Two,
}

/// Parity
/// The "none" state of parity is represented with the Option type (None).
pub enum Parity {
    /// Odd parity
    Odd,

    /// Even parity
    Even,
}

/// A struct holding the configuration for an UART device.
///
/// The `Default` implementation implements the following values:
/// ```ignore
/// # // can't actually create this with the non_exhaustive attribute
/// UartConfig {
///    baudrate: Baud(115_200),
///    data_bits: DataBits::Eight,
///    stop_bits: StopBits::One,
///    parity: None,
///}
/// ```
#[non_exhaustive]
pub struct UartConfig {
    /// The baudrate the uart will run at.
    pub baudrate: Baud,

    /// The amount of data bits the uart should be configured to.
    pub data_bits: DataBits,

    /// The amount of stop bits the uart should be configured to.
    pub stop_bits: StopBits,

    /// The parity that this uart should have
    pub parity: Option<Parity>,
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baudrate: Baud(115_200),
            data_bits: DataBits::Eight,
            stop_bits: StopBits::One,
            parity: None,
        }
    }
}

/// Same as core::convert::Infallible, but implementing serial::Error
///
/// For eh 1.0.0-alpha.6, Infallible doesn't implement serial::Error,
/// so use a locally defined type instead.
/// This should be removed with the next release of e-h.
/// (https://github.com/rust-embedded/embedded-hal/pull/328)
#[cfg(feature = "eh1_0_alpha")]
pub enum SerialInfallible {}

#[cfg(feature = "eh1_0_alpha")]
impl core::fmt::Debug for SerialInfallible {
    fn fmt(&self, _f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {}
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl eh1_0_alpha::serial::Error for SerialInfallible {
    fn kind(&self) -> eh1_0_alpha::serial::ErrorKind {
        match *self {}
    }
}
