//! Useful UART types

use fugit::HertzU32;

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
///
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
    pub baudrate: HertzU32,

    /// The amount of data bits the uart should be configured to.
    pub data_bits: DataBits,

    /// The amount of stop bits the uart should be configured to.
    pub stop_bits: StopBits,

    /// The parity that this uart should have
    pub parity: Option<Parity>,
}

impl UartConfig {
    /// Create a new instance of UartConfig
    pub const fn new(
        baudrate: HertzU32,
        data_bits: DataBits,
        parity: Option<Parity>,
        stop_bits: StopBits,
    ) -> UartConfig {
        UartConfig {
            baudrate,
            data_bits,
            stop_bits,
            parity,
        }
    }
}

impl Default for UartConfig {
    fn default() -> Self {
        Self {
            baudrate: HertzU32::from_raw(115_200),
            data_bits: DataBits::Eight,
            stop_bits: StopBits::One,
            parity: None,
        }
    }
}
