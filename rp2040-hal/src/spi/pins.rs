use super::SpiDevice;
use crate::gpio::{AnyPin, FunctionSpi};
use crate::typelevel::{OptionTSome, Sealed};
use crate::{gpio::bank0::*, typelevel::OptionTNone};
use pac::{SPI0, SPI1};

/// Indicates a valid Rx pin for SPI0 or SPI1
pub trait ValidPinRx<SPI: SpiDevice>: Sealed {}
/// Indicates a valid Tx pin for SPI0 or SPI1
pub trait ValidPinTx<SPI: SpiDevice>: Sealed {}
/// Indicates a valid SCLK pin for SPI0 or SPI1
pub trait ValidPinSck<SPI: SpiDevice>: Sealed {}
/// Indicates a valid CS pin for SPI0 or SPI1
pub trait ValidPinCs<SPI: SpiDevice>: Sealed {}

macro_rules! impl_valid_spi {
    ($($spi:ident: {
        rx: [$($rx:ident),*],
        cs: [$($cs:ident),*],
        sck: [$($sck:ident),*],
        tx: [$($tx:ident),*],
    }),*) => {
        $(
            $(impl ValidPinRx<$spi> for $rx {})*
            $(impl ValidPinTx<$spi> for $tx {})*
            $(impl ValidPinSck<$spi> for $sck {})*
            $(impl ValidPinCs<$spi> for $cs {})*
        )*
    };
}

impl_valid_spi!(
    SPI0: {
        rx: [Gpio0, Gpio4, Gpio16, Gpio20],
        cs: [Gpio1, Gpio5, Gpio17, Gpio21],
        sck: [Gpio2, Gpio6, Gpio18, Gpio22],
        tx: [Gpio3, Gpio7, Gpio19, Gpio23],
    },
    SPI1: {
        rx: [Gpio8, Gpio12, Gpio24, Gpio28],
        cs: [Gpio9, Gpio13, Gpio25, Gpio29],
        sck: [Gpio10, Gpio14, Gpio26],
        tx: [Gpio11, Gpio15, Gpio27],
    }
);

/// Indicates a valid optional Rx pin for SPI0 or SPI1
pub trait ValidOptionRx<SPI: SpiDevice>: Sealed {}
/// Indicates a valid optional Tx pin for SPI0 or SPI1
pub trait ValidOptionTx<SPI: SpiDevice>: Sealed {}
/// Indicates a valid optional SCLK pin for SPI0 or SPI1
pub trait ValidOptionSck<SPI: SpiDevice>: Sealed {}
/// Indicates a valid optional CS pin for SPI0 or SPI1
pub trait ValidOptionCs<SPI: SpiDevice>: Sealed {}
impl<T: SpiDevice> ValidOptionRx<T> for OptionTNone {}
impl<T: SpiDevice> ValidOptionCs<T> for OptionTNone {}
impl<T: SpiDevice> ValidOptionSck<T> for OptionTNone {}
impl<T: SpiDevice> ValidOptionTx<T> for OptionTNone {}

impl<U, T> ValidOptionRx<U> for OptionTSome<T>
where
    U: SpiDevice,
    T: AnyPin<Function = FunctionSpi>,
    T::Id: ValidPinRx<U>,
{
}
impl<U, T> ValidOptionCs<U> for OptionTSome<T>
where
    U: SpiDevice,
    T: AnyPin<Function = FunctionSpi>,
    T::Id: ValidPinCs<U>,
{
}
impl<U, T> ValidOptionSck<U> for OptionTSome<T>
where
    U: SpiDevice,
    T: AnyPin<Function = FunctionSpi>,
    T::Id: ValidPinSck<U>,
{
}
impl<U, T> ValidOptionTx<U> for OptionTSome<T>
where
    U: SpiDevice,
    T: AnyPin<Function = FunctionSpi>,
    T::Id: ValidPinTx<U>,
{
}

/// Declares a valid SPI pinout.
pub trait ValidSpiPinout<U: SpiDevice>: Sealed {
    #[allow(missing_docs)]
    type Rx: ValidOptionRx<U>;
    #[allow(missing_docs)]
    type Cs: ValidOptionCs<U>;
    #[allow(missing_docs)]
    type Sck: ValidOptionSck<U>;
    #[allow(missing_docs)]
    type Tx: ValidOptionTx<U>;
}

impl<Spi, Tx, Sck> ValidSpiPinout<Spi> for (Tx, Sck)
where
    Spi: SpiDevice,
    Tx: AnyPin<Function = FunctionSpi>,
    Sck: AnyPin<Function = FunctionSpi>,
    Tx::Id: ValidPinTx<Spi>,
    Sck::Id: ValidPinSck<Spi>,
{
    type Rx = OptionTNone;
    type Cs = OptionTNone;
    type Sck = OptionTSome<Sck>;
    type Tx = OptionTSome<Tx>;
}

impl<Spi, Tx, Rx, Sck> ValidSpiPinout<Spi> for (Tx, Rx, Sck)
where
    Spi: SpiDevice,
    Tx: AnyPin<Function = FunctionSpi>,
    Sck: AnyPin<Function = FunctionSpi>,
    Rx: AnyPin<Function = FunctionSpi>,
    Tx::Id: ValidPinTx<Spi>,
    Sck::Id: ValidPinSck<Spi>,
    Rx::Id: ValidPinRx<Spi>,
{
    type Rx = OptionTSome<Rx>;
    type Cs = OptionTNone;
    type Sck = OptionTSome<Sck>;
    type Tx = OptionTSome<Tx>;
}
