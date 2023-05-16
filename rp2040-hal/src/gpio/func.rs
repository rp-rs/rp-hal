use core::marker::PhantomData;

use paste::paste;

use super::pin::DynBankId;

pub(crate) mod func_sealed {
    use super::DynFunction;

    pub trait Function {
        fn from(f: DynFunction) -> Self;
        fn as_dyn(&self) -> DynFunction;
    }
    pub trait TypeLevelFunction {}
}

/// Type-level `enum` for pin function.
pub trait Function: func_sealed::Function {}

/// Value-level `enum` for pin function.
#[allow(missing_docs)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynFunction {
    Xip,
    Spi,
    Uart,
    I2c,
    Pwm,
    Sio(DynSioConfig),
    Pio0,
    Pio1,
    Clock,
    Usb,
    Null,
}

/// Value-level `enum` for SIO configuration.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynSioConfig {
    /// Pin is configured as Input.
    Input,
    /// Pin is configured as Output.
    Output,
}

impl Function for DynFunction {}
impl func_sealed::Function for DynFunction {
    fn from(f: DynFunction) -> Self {
        f
    }

    fn as_dyn(&self) -> DynFunction {
        *self
    }
}

macro_rules! pin_func {
    ($($fn:ident $(as $alias:ident)?),*) => {
        $(paste! {
            /// Type-level `variant` for pin [`Function`].
            pub struct [<Function $fn>](pub(super) ());
            impl Function for [<Function $fn>] {}
            impl func_sealed::TypeLevelFunction for [<Function $fn>] {}
            impl func_sealed::Function for [<Function $fn>] {
                fn from(_f: DynFunction) -> Self {
                    Self(())
                }
                fn as_dyn(&self) -> DynFunction {
                    DynFunction::[<$fn>]
                }
            }
            $(
                #[doc = "Alias to [`Function" $fn "`]."]
                pub type [<Function $alias>] = [<Function $fn>];
            )?
        })*
    };
}
pin_func!(Xip, Spi, Uart, I2c as I2C, Pwm, Pio0, Pio1, Clock, Usb, Null);

// =============================
// SIO sub-types

/// Type-level `variant` for pin [`Function`].
pub struct FunctionSio<C>(PhantomData<C>);
impl<C: SioConfig> Function for FunctionSio<C> {}
impl<C: SioConfig> func_sealed::TypeLevelFunction for FunctionSio<C> {}
impl<C: SioConfig> func_sealed::Function for FunctionSio<C> {
    fn from(_f: DynFunction) -> Self {
        FunctionSio(PhantomData)
    }
    fn as_dyn(&self) -> DynFunction {
        DynFunction::Sio(C::DYN)
    }
}
/// Alias to [`FunctionSio<Input>`].
pub type FunctionSioInput = FunctionSio<SioInput>;
/// Alias to [`FunctionSio<Output>`].
pub type FunctionSioOutput = FunctionSio<SioOutput>;

/// Type-level `enum` for SIO configuration.
pub trait SioConfig {
    #[allow(missing_docs)]
    const DYN: DynSioConfig;
}

/// Type-level `variant` for SIO configuration.
pub enum SioInput {}
impl SioConfig for SioInput {
    #[allow(missing_docs)]
    const DYN: DynSioConfig = DynSioConfig::Input;
}
/// Type-level `variant` for SIO configuration.
pub enum SioOutput {}
impl SioConfig for SioOutput {
    #[allow(missing_docs)]
    const DYN: DynSioConfig = DynSioConfig::Output;
}

// =============================
// Pin to function mapping

/// Error type for invalid function conversion.
pub struct InvalidFunction;

/// Marker of valid pin -> function combination.
///
/// Read as `F is a valid function implemented for the pin F`
pub trait ValidFunction<F: Function>: super::pin::PinId {}

impl DynFunction {
    pub(crate) fn is_valid<P: super::pin::PinId>(&self, id: &P) -> bool {
        use DynBankId::*;
        use DynFunction::*;

        let dyn_pin = id.as_dyn();
        match (self, dyn_pin.bank, dyn_pin.num) {
            (Xip, Bank0, _) => false,
            (Clock, _, 0..=19 | 26..=29) => false,
            (_, Bank0, 0..=29) => true,

            (Xip | Sio(_), Qspi, 0..=5) => true,
            (_, Qspi, 0..=5) => false,

            _ => unreachable!(),
        }
    }
}
macro_rules! pin_valid_func {
    ($bank:ident as $prefix:ident, [$head:ident $(, $func:ident)*], [$($name:tt),+]) => {
        pin_valid_func!($bank as $prefix, [$($func),*], [$($name),+]);
        paste::paste!{$(
            impl ValidFunction<[<Function $head>]> for super::pin::[<$bank:lower>]::[<$prefix $name>] {}
        )+}
    };
    ($bank:ident as $prefix:ident, [], [$($name:tt),+]) => {};
}
pin_valid_func!(
    bank0 as Gpio,
    [Spi, Uart, I2c, Pwm, Pio0, Pio1, Usb, Null],
    [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
        25, 26, 27, 28, 29
    ]
);
pin_valid_func!(bank0 as Gpio, [Clock], [20, 21, 22, 23, 24, 25]);
pin_valid_func!(qspi as Qspi, [Xip, Null], [Sclk, Sd0, Sd1, Sd2, Sd3, Ss]);

macro_rules! pin_valid_func_sio {
    ($bank:ident as $prefix:ident, [$($name:tt),+]) => {
        paste::paste!{$(
            impl<C: SioConfig> ValidFunction<FunctionSio<C>> for super::pin::[<$bank:lower>]::[<$prefix $name>] {}
        )+}
    };
}
pin_valid_func_sio!(
    bank0 as Gpio,
    [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
        25, 26, 27, 28, 29
    ]
);
pin_valid_func_sio!(qspi as Qspi, [Sclk, Sd0, Sd1, Sd2, Sd3, Ss]);
