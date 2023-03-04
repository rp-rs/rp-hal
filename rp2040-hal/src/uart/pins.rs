use crate::gpio::{bank0, AnyPin, FunctionUart, OptionalPin};
use crate::pac::{UART0, UART1};
use crate::typelevel::{NoneT, Sealed};

use super::UartDevice;

/// Declares a valid UART pinout.
pub trait ValidUartPinout<UART: UartDevice> {
    #[allow(missing_docs)]
    type Rx: OptionalPin<FunctionUart>;
    #[allow(missing_docs)]
    type Tx: OptionalPin<FunctionUart>;
    #[allow(missing_docs)]
    type Cts: OptionalPin<FunctionUart>;
    #[allow(missing_docs)]
    type Rts: OptionalPin<FunctionUart>;
}

impl<UART, TX, RX, CTS, RTS> ValidUartPinout<UART> for Pins<TX, RX, CTS, RTS>
where
    UART: UartDevice,
    TX: OptionalPin<FunctionUart>,
    RX: OptionalPin<FunctionUart>,
    CTS: OptionalPin<FunctionUart>,
    RTS: OptionalPin<FunctionUart>,
    TX::Id: ValidTxPin<UART>,
    RX::Id: ValidRxPin<UART>,
    CTS::Id: ValidCtsPin<UART>,
    RTS::Id: ValidRtsPin<UART>,
{
    type Rx = RX;
    type Tx = TX;
    type Cts = CTS;
    type Rts = RTS;
}

impl<UART, TX, RX> ValidUartPinout<UART> for (TX, RX)
where
    UART: UartDevice,
    TX: AnyPin<Mode = FunctionUart>,
    RX: AnyPin<Mode = FunctionUart>,
    TX::Id: ValidTxPin<UART>,
    RX::Id: ValidRxPin<UART>,
{
    type Rx = RX;
    type Tx = TX;
    type Cts = NoneT;
    type Rts = NoneT;
}

impl<UART, TX, RX, CTS, RTS> ValidUartPinout<UART> for (TX, RX, CTS, RTS)
where
    UART: UartDevice,
    TX: AnyPin<Mode = FunctionUart>,
    RX: AnyPin<Mode = FunctionUart>,
    CTS: AnyPin<Mode = FunctionUart>,
    RTS: AnyPin<Mode = FunctionUart>,
    TX::Id: ValidTxPin<UART>,
    RX::Id: ValidRxPin<UART>,
    CTS::Id: ValidCtsPin<UART>,
    RTS::Id: ValidRtsPin<UART>,
{
    type Rx = RX;
    type Tx = TX;
    type Cts = CTS;
    type Rts = RTS;
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
/// Every field can be set to [`NoneT`] to not configure them.
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

impl Default for Pins<NoneT, NoneT, NoneT, NoneT> {
    fn default() -> Self {
        Self {
            tx: NoneT,
            rx: NoneT,
            rts: NoneT,
            cts: NoneT,
        }
    }
}

impl<TX, RX, CTS, RTS> Pins<TX, RX, CTS, RTS>
where
    TX: OptionalPin<FunctionUart>,
    RX: OptionalPin<FunctionUart>,
    CTS: OptionalPin<FunctionUart>,
    RTS: OptionalPin<FunctionUart>,
{
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
pub trait ValidTxPin<UART: UartDevice>: Sealed {}
/// Indicates a valid RX pin for UART0 or UART1
pub trait ValidRxPin<UART: UartDevice>: Sealed {}
/// Indicates a valid CTS pin for UART0 or UART1
pub trait ValidCtsPin<UART: UartDevice>: Sealed {}
/// Indicates a valid RTS pin for UART0 or UART1
pub trait ValidRtsPin<UART: UartDevice>: Sealed {}

macro_rules! impl_valid_uart {
    ($($uart:ident: {
        tx: [$($tx:ident),*],
        rx: [$($rx:ident),*],
        cts: [$($cts:ident),*],
        rts: [$($rts:ident),*],
    }),*) => {
        $(
            impl ValidRxPin<$uart> for NoneT {}
            impl ValidTxPin<$uart> for NoneT {}
            impl ValidCtsPin<$uart> for NoneT {}
            impl ValidRtsPin<$uart> for NoneT {}
            $(impl ValidTxPin<$uart> for bank0::$tx {})*
            $(impl ValidRxPin<$uart> for bank0::$rx {})*
            $(impl ValidCtsPin<$uart> for bank0::$cts {})*
            $(impl ValidRtsPin<$uart> for bank0::$rts {})*
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
