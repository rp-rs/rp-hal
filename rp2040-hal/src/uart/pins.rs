use crate::gpio::{bank0::*, AnyPin, FunctionUart};
use crate::pac::{UART0, UART1};
use crate::typelevel::{OptionT, OptionTNone, OptionTSome, Sealed};

use super::UartDevice;

/// Indicates a valid TX pin for UART0 or UART1
pub trait ValidPinTx<UART: UartDevice>: Sealed {}
/// Indicates a valid RX pin for UART0 or UART1
pub trait ValidPinRx<UART: UartDevice>: Sealed {}
/// Indicates a valid CTS pin for UART0 or UART1
pub trait ValidPinCts<UART: UartDevice>: Sealed {}
/// Indicates a valid RTS pin for UART0 or UART1
pub trait ValidPinRts<UART: UartDevice>: Sealed {}

macro_rules! impl_valid_uart {
    ($($uart:ident: {
        tx: [$($tx:ident),*],
        rx: [$($rx:ident),*],
        cts: [$($cts:ident),*],
        rts: [$($rts:ident),*],
    }),*) => {
        $(
            $(impl ValidPinTx<$uart> for $tx {})*
            $(impl ValidPinRx<$uart> for $rx {})*
            $(impl ValidPinCts<$uart> for $cts {})*
            $(impl ValidPinRts<$uart> for $rts {})*
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

/// Indicates a valid optional Tx pin for UART0 or UART1
pub trait ValidOptionTx<U>: OptionT {}
/// Indicates a valid optional Rx pin for UART0 or UART1
pub trait ValidOptionRx<U>: OptionT {}
/// Indicates a valid optional Cts pin for UART0 or UART1
pub trait ValidOptionCts<U>: OptionT {}
/// Indicates a valid optional Rts pin for UART0 or UART1
pub trait ValidOptionRts<U>: OptionT {}
impl<U: UartDevice> ValidOptionTx<U> for OptionTNone {}
impl<U: UartDevice> ValidOptionRx<U> for OptionTNone {}
impl<U: UartDevice> ValidOptionCts<U> for OptionTNone {}
impl<U: UartDevice> ValidOptionRts<U> for OptionTNone {}

impl<U, T> ValidOptionTx<U> for OptionTSome<T>
where
    U: UartDevice,
    T: AnyPin<Function = FunctionUart>,
    T::Id: ValidPinTx<U>,
{
}
impl<U, T> ValidOptionRx<U> for OptionTSome<T>
where
    U: UartDevice,
    T: AnyPin<Function = FunctionUart>,
    T::Id: ValidPinRx<U>,
{
}
impl<U, T> ValidOptionCts<U> for OptionTSome<T>
where
    U: UartDevice,
    T: AnyPin<Function = FunctionUart>,
    T::Id: ValidPinCts<U>,
{
}
impl<U, T> ValidOptionRts<U> for OptionTSome<T>
where
    U: UartDevice,
    T: AnyPin<Function = FunctionUart>,
    T::Id: ValidPinRts<U>,
{
}

/// Declares a valid UART pinout.
pub trait ValidUartPinout<U: UartDevice>: Sealed {
    #[allow(missing_docs)]
    type Rx: ValidOptionRx<U>;
    #[allow(missing_docs)]
    type Tx: ValidOptionTx<U>;
    #[allow(missing_docs)]
    type Cts: ValidOptionCts<U>;
    #[allow(missing_docs)]
    type Rts: ValidOptionRts<U>;
}

impl<Uart, Tx, Rx> ValidUartPinout<Uart> for (Tx, Rx)
where
    Uart: UartDevice,
    Tx: AnyPin<Function = FunctionUart>,
    Rx: AnyPin<Function = FunctionUart>,
    Tx::Id: ValidPinTx<Uart>,
    Rx::Id: ValidPinRx<Uart>,
{
    type Tx = OptionTSome<Tx>;
    type Rx = OptionTSome<Rx>;
    type Cts = OptionTNone;
    type Rts = OptionTNone;
}

impl<Uart, Tx, Rx, Cts, Rts> ValidUartPinout<Uart> for (Tx, Rx, Cts, Rts)
where
    Uart: UartDevice,
    Tx: AnyPin<Function = FunctionUart>,
    Rx: AnyPin<Function = FunctionUart>,
    Cts: AnyPin<Function = FunctionUart>,
    Rts: AnyPin<Function = FunctionUart>,
    Tx::Id: ValidPinTx<Uart>,
    Rx::Id: ValidPinRx<Uart>,
    Cts::Id: ValidPinCts<Uart>,
    Rts::Id: ValidPinRts<Uart>,
{
    type Rx = OptionTSome<Rx>;
    type Tx = OptionTSome<Tx>;
    type Cts = OptionTSome<Cts>;
    type Rts = OptionTSome<Rts>;
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
/// Every field can be set to [`OptionTNone`] to not configure them.
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
///     .tx(gpio_pins.gpio0.into_function())
///     .rx(gpio_pins.gpio1.into_function());
///
/// fn assert_is_valid_uart0<T: ValidUartPinout<UART0>>(_: T) {}
///
/// assert_is_valid_uart0(pins);
/// ```
#[allow(missing_docs)]
pub struct Pins<Tx, Rx, Cts, Rts> {
    pub tx: Tx,
    pub rx: Rx,
    pub cts: Cts,
    pub rts: Rts,
}

impl Default for Pins<OptionTNone, OptionTNone, OptionTNone, OptionTNone> {
    fn default() -> Self {
        Self {
            tx: OptionTNone,
            rx: OptionTNone,
            rts: OptionTNone,
            cts: OptionTNone,
        }
    }
}

impl<Tx, Rx, Cts, Rts> Pins<Tx, Rx, Cts, Rts> {
    /// Set the TX pin
    pub fn tx<NewTx>(self, tx: NewTx) -> Pins<OptionTSome<NewTx>, Rx, Cts, Rts> {
        Pins {
            tx: OptionTSome(tx),
            rx: self.rx,
            rts: self.rts,
            cts: self.cts,
        }
    }
    /// Set the RX pin
    pub fn rx<NewRx>(self, rx: NewRx) -> Pins<Tx, OptionTSome<NewRx>, Cts, Rts> {
        Pins {
            tx: self.tx,
            rx: OptionTSome(rx),
            rts: self.rts,
            cts: self.cts,
        }
    }
    /// Set the CTS pin
    pub fn cts<NewCts>(self, cts: NewCts) -> Pins<Tx, Rx, OptionTSome<NewCts>, Rts> {
        Pins {
            tx: self.tx,
            rx: self.rx,
            rts: self.rts,
            cts: OptionTSome(cts),
        }
    }
    /// Set the RTS pin
    pub fn rts<NewRts>(self, rts: NewRts) -> Pins<Tx, Rx, Cts, OptionTSome<NewRts>> {
        Pins {
            tx: self.tx,
            rx: self.rx,
            rts: OptionTSome(rts),
            cts: self.cts,
        }
    }
}

impl<Tx, Rx, Cts, Rts> Sealed for Pins<Tx, Rx, Cts, Rts> {}
impl<Uart, Tx, Rx, Cts, Rts> ValidUartPinout<Uart> for Pins<Tx, Rx, Cts, Rts>
where
    Uart: UartDevice,
    Tx: ValidOptionTx<Uart>,
    Rx: ValidOptionRx<Uart>,
    Cts: ValidOptionCts<Uart>,
    Rts: ValidOptionRts<Uart>,
{
    type Rx = Rx;
    type Tx = Tx;
    type Cts = Cts;
    type Rts = Rts;
}
