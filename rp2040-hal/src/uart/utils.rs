use crate::pac::{uart0::RegisterBlock, UART0, UART1};
use crate::resets::SubsystemReset;
use crate::typelevel::Sealed;
use core::ops::Deref;
use fugit::HertzU32;
use rp2040_pac::dma::ch::ch_ctrl_trig::TREQ_SEL_A;

/// Error type for UART operations.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Bad argument : when things overflow, ...
    BadArgument,
}
/// State of the UART Peripheral.
pub trait State: Sealed {}

/// Trait to handle both underlying devices (UART0 & UART1)
pub trait UartDevice: Deref<Target = RegisterBlock> + SubsystemReset + Sealed + 'static {
    /// The DREQ number for which TX DMA requests are triggered.
    fn tx_dreq() -> u8
    where
        Self: Sized;
    /// The DREQ number for which RX DMA requests are triggered.
    fn rx_dreq() -> u8
    where
        Self: Sized;
}

impl UartDevice for UART0 {
    /// The DREQ number for which TX DMA requests are triggered.
    fn tx_dreq() -> u8 {
        TREQ_SEL_A::UART0_TX.into()
    }
    /// The DREQ number for which RX DMA requests are triggered.
    fn rx_dreq() -> u8 {
        TREQ_SEL_A::UART0_RX.into()
    }
}
impl Sealed for UART0 {}
impl UartDevice for UART1 {
    /// The DREQ number for which TX DMA requests are triggered.
    fn tx_dreq() -> u8 {
        TREQ_SEL_A::UART1_TX.into()
    }
    /// The DREQ number for which RX DMA requests are triggered.
    fn rx_dreq() -> u8 {
        TREQ_SEL_A::UART1_RX.into()
    }
}
impl Sealed for UART1 {}

/// UART is enabled.
pub struct Enabled;

/// UART is disabled.
pub struct Disabled;

impl State for Enabled {}
impl Sealed for Enabled {}
impl State for Disabled {}
impl Sealed for Disabled {}

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

/// Rx/Tx FIFO Watermark
///
/// Determine the FIFO level that trigger DMA/Interrupt
/// Default is Bytes16, see DS Table 423 and UARTIFLS Register
/// Example of use:
///     uart0.set_fifos(true); // Default is false
///     uart0.set_rx_watermark(hal::uart::FifoWatermark::Bytes8);
///     uart0.enable_rx_interrupt();
///
pub enum FifoWatermark {
    /// Trigger when 4 bytes are (Rx: filled / Tx: available)
    Bytes4,
    /// Trigger when 8 bytes are (Rx: filled / Tx: available)
    Bytes8,
    /// Trigger when 16 bytes are (Rx: filled / Tx: available)
    Bytes16,
    /// Trigger when 24 bytes are (Rx: filled / Tx: available)
    Bytes24,
    /// Trigger when 28 bytes are (Rx: filled / Tx: available)
    Bytes28,
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
