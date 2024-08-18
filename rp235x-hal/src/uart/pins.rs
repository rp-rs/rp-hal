use core::marker::PhantomData;

use crate::gpio::{
    bank0::*, pin::pin_sealed::TypeLevelPinId, AnyPin, DynFunction, FunctionUart, FunctionUartAux,
    PullType,
};
use crate::pac::{UART0, UART1};
use crate::typelevel::{OptionT, OptionTNone, OptionTSome, Sealed};

use super::UartDevice;

// All type level checked pins are inherently valid.
macro_rules! pin_validation {
    ($p:ident) => {
        paste::paste!{
            #[doc = "Indicates a valid " $p " pin the given UART"]
            pub trait [<ValidPin $p>]<UART: UartDevice>: Sealed {}

            #[doc = "A runtime validated " $p " pin for the given UART."]
            pub struct [<ValidatedPin $p>]<P, Uart>(P, PhantomData<Uart>);

            impl<P, UART: UartDevice> Sealed for [<ValidatedPin $p>]<P, UART> {}

            impl<P, U> [<ValidatedPin $p>]<P, U>
            where
                P: AnyPin<Function = DynFunction>,
                U: UartDevice,
            {
                /// Validate a pin's function on a uart peripheral.
                ///
                #[doc = "Will err if the pin cannot be used as a " $p " pin for that Uart."]
                pub fn validate(p: P, _u: &U) -> Result<Self, P> {
                    if [<$p:upper>].contains(&(p.borrow().id().num, U::ID, p.borrow().function())) &&
                        p.borrow().id().bank == crate::gpio::DynBankId::Bank0 {
                        Ok(Self(p, PhantomData))
                    } else {
                        Err(p)
                    }
                }
            }

            #[doc = "Indicates a valid optional " $p " pin for UART0 or UART1"]
            pub trait [<ValidOption $p>]<U>: OptionT {}

            impl<U: UartDevice> [<ValidOption $p>]<U> for OptionTNone {}
            impl<U, T> [<ValidOption $p>]<U> for OptionTSome<T>
            where
                U: UartDevice,
                T: [<ValidPin $p>]<U>,
            {
            }
        }
    };
    ($($p:ident),*) => {
        $(
            pin_validation!($p);
         )*
    };
}
pin_validation!(Tx, Rx, Cts, Rts);

impl<P> ValidPinTx<UART0> for crate::gpio::Pin<Gpio0, FunctionUart, P> where P: PullType {}
impl<P> ValidPinTx<UART0> for crate::gpio::Pin<Gpio2, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinTx<UART0> for crate::gpio::Pin<Gpio12, FunctionUart, P> where P: PullType {}
impl<P> ValidPinTx<UART0> for crate::gpio::Pin<Gpio14, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinTx<UART0> for crate::gpio::Pin<Gpio16, FunctionUart, P> where P: PullType {}
impl<P> ValidPinTx<UART0> for crate::gpio::Pin<Gpio18, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinTx<UART0> for crate::gpio::Pin<Gpio28, FunctionUart, P> where P: PullType {}

impl<P> ValidPinTx<UART1> for crate::gpio::Pin<Gpio4, FunctionUart, P> where P: PullType {}
impl<P> ValidPinTx<UART1> for crate::gpio::Pin<Gpio6, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinTx<UART1> for crate::gpio::Pin<Gpio8, FunctionUart, P> where P: PullType {}
impl<P> ValidPinTx<UART1> for crate::gpio::Pin<Gpio10, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinTx<UART1> for crate::gpio::Pin<Gpio20, FunctionUart, P> where P: PullType {}
impl<P> ValidPinTx<UART1> for crate::gpio::Pin<Gpio22, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinTx<UART1> for crate::gpio::Pin<Gpio24, FunctionUart, P> where P: PullType {}
impl<P> ValidPinTx<UART1> for crate::gpio::Pin<Gpio26, FunctionUartAux, P> where P: PullType {}

impl<P, UART> ValidPinTx<UART> for ValidatedPinTx<P, UART>
where
    P: AnyPin,
    UART: UartDevice,
{
}

impl<P> ValidPinRx<UART0> for crate::gpio::Pin<Gpio1, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRx<UART0> for crate::gpio::Pin<Gpio3, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinRx<UART0> for crate::gpio::Pin<Gpio13, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRx<UART0> for crate::gpio::Pin<Gpio15, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinRx<UART0> for crate::gpio::Pin<Gpio17, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRx<UART0> for crate::gpio::Pin<Gpio19, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinRx<UART0> for crate::gpio::Pin<Gpio29, FunctionUart, P> where P: PullType {}

impl<P> ValidPinRx<UART1> for crate::gpio::Pin<Gpio5, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRx<UART1> for crate::gpio::Pin<Gpio7, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinRx<UART1> for crate::gpio::Pin<Gpio9, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRx<UART1> for crate::gpio::Pin<Gpio11, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinRx<UART1> for crate::gpio::Pin<Gpio21, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRx<UART1> for crate::gpio::Pin<Gpio23, FunctionUartAux, P> where P: PullType {}
impl<P> ValidPinRx<UART1> for crate::gpio::Pin<Gpio25, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRx<UART1> for crate::gpio::Pin<Gpio27, FunctionUartAux, P> where P: PullType {}

impl<P, UART> ValidPinRx<UART> for ValidatedPinRx<P, UART>
where
    P: AnyPin,
    UART: UartDevice,
{
}

impl<P> ValidPinCts<UART0> for crate::gpio::Pin<Gpio2, FunctionUart, P> where P: PullType {}
impl<P> ValidPinCts<UART0> for crate::gpio::Pin<Gpio14, FunctionUart, P> where P: PullType {}
impl<P> ValidPinCts<UART0> for crate::gpio::Pin<Gpio18, FunctionUart, P> where P: PullType {}

impl<P> ValidPinCts<UART1> for crate::gpio::Pin<Gpio6, FunctionUart, P> where P: PullType {}
impl<P> ValidPinCts<UART1> for crate::gpio::Pin<Gpio10, FunctionUart, P> where P: PullType {}
impl<P> ValidPinCts<UART1> for crate::gpio::Pin<Gpio22, FunctionUart, P> where P: PullType {}
impl<P> ValidPinCts<UART1> for crate::gpio::Pin<Gpio26, FunctionUart, P> where P: PullType {}

impl<P, UART> ValidPinCts<UART> for ValidatedPinCts<P, UART>
where
    P: AnyPin,
    UART: UartDevice,
{
}
impl<P> ValidPinRts<UART0> for crate::gpio::Pin<Gpio3, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRts<UART0> for crate::gpio::Pin<Gpio15, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRts<UART0> for crate::gpio::Pin<Gpio19, FunctionUart, P> where P: PullType {}

impl<P> ValidPinRts<UART1> for crate::gpio::Pin<Gpio7, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRts<UART1> for crate::gpio::Pin<Gpio11, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRts<UART1> for crate::gpio::Pin<Gpio23, FunctionUart, P> where P: PullType {}
impl<P> ValidPinRts<UART1> for crate::gpio::Pin<Gpio27, FunctionUart, P> where P: PullType {}

impl<P, UART> ValidPinRts<UART> for ValidatedPinRts<P, UART>
where
    P: AnyPin,
    UART: UartDevice,
{
}

const TX: &[(u8, usize, DynFunction)] = &[
    (Gpio0::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio12::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio16::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio28::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio4::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio8::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio20::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio24::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio2::ID.num, UART0::ID, DynFunction::UartAux),
    (Gpio14::ID.num, UART0::ID, DynFunction::UartAux),
    (Gpio18::ID.num, UART0::ID, DynFunction::UartAux),
    (Gpio6::ID.num, UART1::ID, DynFunction::UartAux),
    (Gpio10::ID.num, UART1::ID, DynFunction::UartAux),
    (Gpio22::ID.num, UART1::ID, DynFunction::UartAux),
    (Gpio26::ID.num, UART1::ID, DynFunction::UartAux),
];

const RX: &[(u8, usize, DynFunction)] = &[
    (Gpio1::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio13::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio17::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio29::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio5::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio9::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio21::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio25::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio3::ID.num, UART0::ID, DynFunction::UartAux),
    (Gpio15::ID.num, UART0::ID, DynFunction::UartAux),
    (Gpio19::ID.num, UART0::ID, DynFunction::UartAux),
    (Gpio7::ID.num, UART1::ID, DynFunction::UartAux),
    (Gpio11::ID.num, UART1::ID, DynFunction::UartAux),
    (Gpio23::ID.num, UART1::ID, DynFunction::UartAux),
    (Gpio27::ID.num, UART1::ID, DynFunction::UartAux),
];

const CTS: &[(u8, usize, DynFunction)] = &[
    (Gpio2::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio14::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio18::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio6::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio10::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio22::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio26::ID.num, UART1::ID, DynFunction::Uart),
];

const RTS: &[(u8, usize, DynFunction)] = &[
    (Gpio3::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio15::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio19::ID.num, UART0::ID, DynFunction::Uart),
    (Gpio7::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio11::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio23::ID.num, UART1::ID, DynFunction::Uart),
    (Gpio27::ID.num, UART1::ID, DynFunction::Uart),
];

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
    Tx: ValidPinTx<Uart>,
    Rx: ValidPinRx<Uart>,
{
    type Tx = OptionTSome<Tx>;
    type Rx = OptionTSome<Rx>;
    type Cts = OptionTNone;
    type Rts = OptionTNone;
}

impl<Uart, Tx, Rx, Cts, Rts> ValidUartPinout<Uart> for (Tx, Rx, Cts, Rts)
where
    Uart: UartDevice,
    Tx: ValidPinTx<Uart>,
    Rx: ValidPinRx<Uart>,
    Cts: ValidPinCts<Uart>,
    Rts: ValidPinRts<Uart>,
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
/// |UART |     TX      |     RX      |CTS (or TX in Aux mode)|RTS (or RX in Aux mode)|
/// |-----|-------------|-------------|-----------------------|-----------------------|
/// |UART0|0, 12, 16, 28|1, 13, 17, 29|2, 14, 18              |3, 15, 19              |
/// |UART1|4, 8, 20, 24 |5, 9, 21, 25 |6, 10, 22, 26          |7, 11, 23, 27          |
///
/// The RP235x allows you to use CTS pins as TX pins by using the
/// `FunctionUartAux` pin function (instead of `FunctionUart`). The same goes
/// for using RTS pins as RX pins.
///
/// Every field can be set to [`OptionTNone`] to not configure them.
///
/// Note that you can also use tuples `(RX, TX)` or `(RX, TX, CTS, RTS)` instead of this type.
///
/// This struct can either be filled manually or with a builder pattern:
///
/// ```no_run
/// # use rp235x_hal::uart::{Pins, ValidUartPinout};
/// # use rp235x_hal::pac::UART0;
/// # let gpio_pins: rp235x_hal::gpio::Pins = unsafe { core::mem::zeroed() };
/// let pins = Pins::default()
///     .tx(gpio_pins.gpio0.into_function())
///     .rx(gpio_pins.gpio1.into_function());
///
/// fn assert_is_valid_uart0<T: ValidUartPinout<UART0>>(_: T) {}
///
/// assert_is_valid_uart0(pins);
/// ```
pub struct Pins<Tx, Rx, Cts, Rts> {
    #[allow(missing_docs)]
    pub tx: Tx,
    #[allow(missing_docs)]
    pub rx: Rx,
    #[allow(missing_docs)]
    pub cts: Cts,
    #[allow(missing_docs)]
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
