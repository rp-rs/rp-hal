use core::marker::PhantomData;

use paste::paste;

use super::pin::DynBankId;

pub(crate) mod func_sealed {
    use super::DynFunction;

    pub trait Function {
        fn from(f: DynFunction) -> Self;
        fn as_dyn(&self) -> DynFunction;
    }
}

/// Type-level `enum` for pin function.
pub trait Function: func_sealed::Function {}

/// Describes the function currently assigned to a pin with a dynamic type.
///
/// A 'pin' on the RP2040 can be connected to different parts of the chip
/// internally - for example, it could be configured as a GPIO pin and connected
/// to the SIO block, or it could be configured as a UART pin and connected to
/// the UART block.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DynFunction {
    /// The 'XIP' (or Execute-in-place) function, which means talking to the QSPI Flash.
    Xip,
    /// The 'SPI' (or serial-peripheral-interface) function.
    Spi,
    /// The 'UART' (or serial-port) function.
    Uart,
    /// The 'I2C' (or inter-integrated circuit) function. This is sometimes also called TWI (for
    /// two-wire-interface).
    I2c,
    /// The 'PWM' (or pulse-width-modulation) function.
    Pwm,
    /// The 'SIO' (or single-cycle input-output) function. This is the function to use for
    /// 'manually' controlling the GPIO.
    Sio(DynSioConfig),
    /// The 'PIO' (or programmable-input-output) function for the PIO0 peripheral block.
    Pio0,
    /// The 'PIO' (or programmable-input-output) function for the PIO1 peripheral block.
    Pio1,
    /// The 'Clock' function. This can be used to input or output clock references to or from the
    /// rp2040.
    Clock,
    /// The 'USB' function. Only VBUS detect, VBUS enable and overcurrent detect are configurable.
    /// Other USB io have dedicated pins.
    Usb,
    /// The 'Null' function for unused pins.
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
    #[inline]
    fn from(f: DynFunction) -> Self {
        f
    }

    #[inline]
    fn as_dyn(&self) -> DynFunction {
        *self
    }
}

macro_rules! pin_func {
    ($($fn:ident $(as $alias:ident)?),*) => {
        $(paste! {
            /// Type-level `variant` for pin [`Function`].
            #[derive(Debug)]
            pub struct [<Function $fn>](pub(super) ());
            impl Function for [<Function $fn>] {}
            impl func_sealed::Function for [<Function $fn>] {
                #[inline]
                fn from(_f: DynFunction) -> Self {
                    Self(())
                }
                #[inline]
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

//==============================================================================
// SIO sub-types
//==============================================================================

/// Type-level `variant` for pin [`Function`].
#[derive(Debug)]
pub struct FunctionSio<C>(PhantomData<C>);
impl<C: SioConfig> Function for FunctionSio<C> {}
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
#[derive(Debug)]
pub enum SioInput {}
impl SioConfig for SioInput {
    #[allow(missing_docs)]
    const DYN: DynSioConfig = DynSioConfig::Input;
}
/// Type-level `variant` for SIO configuration.
#[derive(Debug)]
pub enum SioOutput {}
impl SioConfig for SioOutput {
    #[allow(missing_docs)]
    const DYN: DynSioConfig = DynSioConfig::Output;
}

//==============================================================================
// Pin to function mapping
//==============================================================================

/// Error type for invalid function conversion.
pub struct InvalidFunction;

/// Marker of valid pin -> function combination.
///
/// Where `impl ValidFunction<F> for I` reads as `F is a valid function implemented for the pin I`.
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
impl<P: super::pin::PinId> ValidFunction<DynFunction> for P {}
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
