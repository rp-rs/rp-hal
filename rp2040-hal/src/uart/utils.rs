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

/// When there's a read error.
pub struct ReadError<'err> {
    /// The type of error
    pub err_type: ReadErrorType,

    /// Reference to the data that was read but eventually discared because of the error.
    pub discared: &'err [u8],
}

/// Possible types of read errors. See Chapter 4, Section 2 §8 - Table 436: "UARTDR Register"
#[cfg_attr(feature = "eh1_0_alpha", derive(Debug))]
pub enum ReadErrorType {
    /// Triggered when the FIFO (or shift-register) is overflowed.
    Overrun,

    /// Triggered when a break is received
    Break,

    /// Triggered when there is a parity mismatch between what's received and our settings.
    Parity,

    /// Triggered when the received character didn't have a valid stop bit.
    Framing,
}

#[cfg(feature = "eh1_0_alpha")]
impl eh1_0_alpha::serial::Error for ReadErrorType {
    fn kind(&self) -> eh1_0_alpha::serial::ErrorKind {
        match self {
            ReadErrorType::Overrun => eh1_0_alpha::serial::ErrorKind::Overrun,
            ReadErrorType::Break => eh1_0_alpha::serial::ErrorKind::Other,
            ReadErrorType::Parity => eh1_0_alpha::serial::ErrorKind::Parity,
            ReadErrorType::Framing => eh1_0_alpha::serial::ErrorKind::FrameFormat,
        }
    }
}

/// State of the UART Peripheral.
pub trait State {}

/// Trait to handle both underlying devices (UART0 & UART1)
pub trait UartDevice: Deref<Target = RegisterBlock> + SubsystemReset {}

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
