use crate::gpio::{bank0, FunctionUart, Pin};
use crate::pac::{UART0, UART1};

use super::UartDevice;

/// Declares a valid UART pinout.
pub trait ValidUartPinout<UART: UartDevice> {
    /// Indicates TX should be enabled for this pinout
    const TX_ENABLED: bool;
    /// Indicates RX should be enabled for this pinout
    const RX_ENABLED: bool;
    /// Indicates CTS should be enabled for this pinout
    const CTS_ENABLED: bool;
    /// Indicates RTS should be enabled for this pinout
    const RTS_ENABLED: bool;
}

impl<UART, TX, RX, CTS, RTS> ValidUartPinout<UART> for Pins<TX, RX, CTS, RTS>
where
    UART: UartDevice,
    TX: Tx<UART>,
    RX: Rx<UART>,
    CTS: Cts<UART>,
    RTS: Rts<UART>,
{
    const TX_ENABLED: bool = TX::ENABLED;
    const RX_ENABLED: bool = RX::ENABLED;
    const CTS_ENABLED: bool = CTS::ENABLED;
    const RTS_ENABLED: bool = RTS::ENABLED;
}

impl<UART, TX, RX> ValidUartPinout<UART> for (TX, RX)
where
    UART: UartDevice,
    TX: Tx<UART>,
    RX: Rx<UART>,
{
    const TX_ENABLED: bool = TX::ENABLED;
    const RX_ENABLED: bool = RX::ENABLED;
    const CTS_ENABLED: bool = false;
    const RTS_ENABLED: bool = false;
}

impl<UART, TX, RX, CTS, RTS> ValidUartPinout<UART> for (TX, RX, CTS, RTS)
where
    UART: UartDevice,
    TX: Tx<UART>,
    RX: Rx<UART>,
    CTS: Cts<UART>,
    RTS: Rts<UART>,
{
    const TX_ENABLED: bool = TX::ENABLED;
    const RX_ENABLED: bool = RX::ENABLED;
    const CTS_ENABLED: bool = CTS::ENABLED;
    const RTS_ENABLED: bool = RTS::ENABLED;
}

/// Customizable Uart pinout, allowing you to set the pins individually.
///
/// The following pins are valid UART pins:
///
/// |UART |     TX      |     RX      |    CTS      |    RTS      |
/// |-----|-------------|-------------|-------------|-------------|
/// |UART0|0, 12, 16, 28|1, 13, 17, 29|2, 14, 18    |3, 15, 19    |
/// |UART1|4, 8, 20, 24 |5, 9, 21, 25 |6, 10, 22, 26|7, 11, 23, 27|
///
/// Every field can be set to `()` to not configure them.
///
/// Note that you can also use tuples `(RX, TX)` or `(RX, TX, CTS, RTS)` instead of this type.
///
/// This struct can either be filled manually or with a builder pattern:
///
/// ```no_run
/// # use rp2040_hal::uart::{Pins, ValidUartPinout};
/// # use rp2040_hal::pac::UART0;
/// # let gpio_pins: rp2040_hal::gpio::Pins = unsafe { core::mem::zeroed() };
/// let pins = Pins::default()
///     .tx(gpio_pins.gpio0.into_mode())
///     .rx(gpio_pins.gpio1.into_mode());
///
/// fn assert_is_valid_uart0<T: ValidUartPinout<UART0>>(_: T) {}
///
/// assert_is_valid_uart0(pins);
/// ```
#[allow(missing_docs)]
pub struct Pins<TX, RX, CTS, RTS> {
    pub tx: TX,
    pub rx: RX,
    pub rts: RTS,
    pub cts: CTS,
}

impl Default for Pins<(), (), (), ()> {
    fn default() -> Self {
        Self {
            tx: (),
            rx: (),
            rts: (),
            cts: (),
        }
    }
}

impl<TX, RX, CTS, RTS> Pins<TX, RX, CTS, RTS> {
    /// Set the TX pin
    pub fn tx<NTX>(self, tx: NTX) -> Pins<NTX, RX, CTS, RTS> {
        Pins {
            tx,
            rx: self.rx,
            rts: self.rts,
            cts: self.cts,
        }
    }
    /// Set the RX pin
    pub fn rx<NRX>(self, rx: NRX) -> Pins<TX, NRX, CTS, RTS> {
        Pins {
            tx: self.tx,
            rx,
            rts: self.rts,
            cts: self.cts,
        }
    }
    /// Set the CTS pin
    pub fn cts<NCTS>(self, cts: NCTS) -> Pins<TX, RX, NCTS, RTS> {
        Pins {
            tx: self.tx,
            rx: self.rx,
            rts: self.rts,
            cts,
        }
    }
    /// Set the RTS pin
    pub fn rts<NRTS>(self, rts: NRTS) -> Pins<TX, RX, CTS, NRTS> {
        Pins {
            tx: self.tx,
            rx: self.rx,
            rts,
            cts: self.cts,
        }
    }
}

/// Indicates a valid TX pin for UART0 or UART1
pub trait Tx<UART: UartDevice> {
    #[allow(missing_docs)]
    const ENABLED: bool;
}
/// Indicates a valid RX pin for UART0 or UART1
pub trait Rx<UART: UartDevice> {
    #[allow(missing_docs)]
    const ENABLED: bool;
}
/// Indicates a valid CTS pin for UART0 or UART1
pub trait Cts<UART: UartDevice> {
    #[allow(missing_docs)]
    const ENABLED: bool;
}
/// Indicates a valid RTS pin for UART0 or UART1
pub trait Rts<UART: UartDevice> {
    #[allow(missing_docs)]
    const ENABLED: bool;
}

impl<UART: UartDevice> Tx<UART> for () {
    const ENABLED: bool = false;
}
impl<UART: UartDevice> Rx<UART> for () {
    const ENABLED: bool = false;
}
impl<UART: UartDevice> Cts<UART> for () {
    const ENABLED: bool = false;
}
impl<UART: UartDevice> Rts<UART> for () {
    const ENABLED: bool = false;
}

macro_rules! impl_valid_uart {
    ($($uart:ident: {
        tx: [$($tx:ident),*],
        rx: [$($rx:ident),*],
        cts: [$($cts:ident),*],
        rts: [$($rts:ident),*],
    }),*) => {
        $(
            $(
                impl Tx<$uart> for Pin<bank0::$tx, FunctionUart> {
                    const ENABLED: bool = true;
                }
            )*
            $(
                impl Rx<$uart> for Pin<bank0::$rx, FunctionUart> {
                    const ENABLED: bool = true;
                }
            )*
            $(
                impl Cts<$uart> for Pin<bank0::$cts, FunctionUart> {
                    const ENABLED: bool = true;
                }
            )*
            $(
                impl Rts<$uart> for Pin<bank0::$rts, FunctionUart> {
                    const ENABLED: bool = true;
                }
            )*
        )*
    };
}

impl_valid_uart!(
    UART0: {
        tx: [Gpio0, Gpio12, Gpio16, Gpio28],
        rx: [Gpio1, Gpio13, Gpio17, Gpio29],
        cts: [Gpio2, Gpio14, Gpio18],
        rts: [Gpio3, Gpio15, Gpio19],
    },
    UART1: {
        tx: [Gpio4, Gpio8, Gpio20, Gpio24],
        rx: [Gpio5, Gpio9, Gpio21, Gpio25],
        cts: [Gpio6, Gpio10, Gpio22, Gpio26],
        rts: [Gpio7, Gpio11, Gpio23, Gpio27],
    }
);
