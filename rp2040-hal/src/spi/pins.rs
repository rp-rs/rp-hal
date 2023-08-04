use core::marker::PhantomData;

use crate::{
    gpio::{bank0::*, pin::pin_sealed::TypeLevelPinId, AnyPin, FunctionSpi},
    pac::{SPI0, SPI1},
    typelevel::{OptionT, OptionTNone, OptionTSome, Sealed},
};

use super::SpiDevice;

macro_rules! pin_validation {
    ($p:ident) => {
        paste::paste!{
            #[doc = "Indicates a valid " $p " pin for SPI0 or SPI1"]
            pub trait [<ValidPinId $p>]<SPI: SpiDevice>: Sealed {}

            #[doc = "Indicates a valid " $p " pin for SPI0 or SPI1"]
            pub trait [<ValidPin $p>]<SPI: SpiDevice>: Sealed {}

            impl<T, U: SpiDevice> [<ValidPin $p>]<U> for T
            where
                T: AnyPin<Function = FunctionSpi>,
                T::Id: [<ValidPinId $p>]<U>,
            {
            }

            #[doc = "A runtime validated " $p " pin for spi."]
            pub struct [<ValidatedPin $p>]<P, Spi>(P, PhantomData<Spi>);
            impl<P, SPI: SpiDevice> Sealed for [<ValidatedPin $p>]<P, SPI> {}
            impl<P, SPI: SpiDevice> [<ValidPin $p>]<SPI> for [<ValidatedPin $p>]<P, SPI> {}
            impl<P, S> [<ValidatedPin $p>]<P, S>
            where
                P: AnyPin<Function = FunctionSpi>,
                S: SpiDevice,
            {
                /// Validate a pin's function on a spi peripheral.
                ///
                #[doc = "Will err if the pin cannot be used as a " $p " pin for that Spi."]
                pub fn validate(p: P, _u: &S) -> Result<Self, P> {
                    if [<$p:upper>].contains(&(p.borrow().id().num, S::ID)) &&
                        p.borrow().id().bank == crate::gpio::DynBankId::Bank0 {
                        Ok(Self(p, PhantomData))
                    } else {
                        Err(p)
                    }
                }
            }

            #[doc = "Indicates a valid optional " $p " pin for SPI0 or SPI1"]
            pub trait [<ValidOption $p>]<U>: OptionT {}

            impl<U: SpiDevice> [<ValidOption $p>]<U> for OptionTNone {}
            impl<U, T> [<ValidOption $p>]<U> for OptionTSome<T>
            where
                U: SpiDevice,
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
pin_validation!(Tx, Rx, Sck, Cs);

macro_rules! impl_valid_spi {
    ($($spi:ident: {
        rx: [$($rx:ident),*],
        cs: [$($cs:ident),*],
        sck: [$($sck:ident),*],
        tx: [$($tx:ident),*],
    }),*) => {
        $(
            $(impl ValidPinIdRx<$spi> for $rx {})*
            $(impl ValidPinIdTx<$spi> for $tx {})*
            $(impl ValidPinIdSck<$spi> for $sck {})*
            $(impl ValidPinIdCs<$spi> for $cs {})*
        )*

        const RX: &[(u8, usize)] = &[$($(($rx::ID.num, $spi::ID)),*),*];
        const TX: &[(u8, usize)] = &[$($(($tx::ID.num, $spi::ID)),*),*];
        const SCK: &[(u8, usize)] = &[$($(($sck::ID.num, $spi::ID)),*),*];
        const CS: &[(u8, usize)] = &[$($(($cs::ID.num, $spi::ID)),*),*];
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
    Tx: ValidPinTx<Spi>,
    Sck: ValidPinSck<Spi>,
{
    type Rx = OptionTNone;
    type Cs = OptionTNone;
    type Sck = OptionTSome<Sck>;
    type Tx = OptionTSome<Tx>;
}

impl<Spi, Tx, Rx, Sck> ValidSpiPinout<Spi> for (Tx, Rx, Sck)
where
    Spi: SpiDevice,
    Tx: ValidPinTx<Spi>,
    Sck: ValidPinSck<Spi>,
    Rx: ValidPinRx<Spi>,
{
    type Rx = OptionTSome<Rx>;
    type Cs = OptionTNone;
    type Sck = OptionTSome<Sck>;
    type Tx = OptionTSome<Tx>;
}
