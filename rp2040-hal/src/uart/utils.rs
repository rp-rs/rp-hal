use crate::pac::dma::ch::ch_ctrl_trig::TREQ_SEL_A;
use crate::pac::{uart0::RegisterBlock, UART0, UART1};
use crate::resets::SubsystemReset;
use crate::typelevel::Sealed;
use core::ops::Deref;

#[doc(inline)]
pub use rp_hal_common::uart::{DataBits, Parity, StopBits, UartConfig};

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
    /// Index of the Uart.
    const ID: usize;

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
    const ID: usize = 0;

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
    const ID: usize = 1;

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
