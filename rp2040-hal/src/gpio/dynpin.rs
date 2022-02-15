//! # Type-erased, value-level module for GPIO pins
//!
//! Based heavily on `atsamd-hal`.
//!
//! Although the type-level API is generally preferred, it is not suitable in
//! all cases. Because each pin is represented by a distinct type, it is not
//! possible to store multiple pins in a homogeneous data structure. The
//! value-level API solves this problem by erasing the type information and
//! tracking the pin at run-time.
//!
//! Value-level pins are represented by the [`DynPin`] type. [`DynPin`] has two
//! fields, `id` and `mode` with types [`DynPinId`] and [`DynPinMode`]
//! respectively. The implementation of these types closely mirrors the
//! type-level API.
//!
//! Instances of [`DynPin`] cannot be created directly. Rather, they must be
//! created from their type-level equivalents using [`From`]/[`Into`].
//!
//! ```no_run
//! // Move a pin out of the Pins struct and convert to a DynPin
//! # use rp2040_hal::{pac, gpio::{DynPin, bank0::Gpio12, Pins}, Sio};
//! # let mut peripherals = pac::Peripherals::take().unwrap();
//! # let sio = Sio::new(peripherals.SIO);
//! # let pins = Pins::new(peripherals.IO_BANK0,peripherals.PADS_BANK0,sio.gpio_bank0, &mut peripherals.RESETS);
//! # use rp2040_hal::gpio::DYN_FLOATING_INPUT;
//! let gpio12: DynPin = pins.gpio12.into();
//! ```
//!
//! Conversions between pin modes use a value-level version of the type-level
//! API.
//!
//! ```no_run
//! # use rp2040_hal::{pac, gpio::{DynPin, Pins}, Sio};
//! # let mut peripherals = pac::Peripherals::take().unwrap();
//! # let sio = Sio::new(peripherals.SIO);
//! # let pins = Pins::new(peripherals.IO_BANK0,peripherals.PADS_BANK0,sio.gpio_bank0, &mut peripherals.RESETS);
//! # use rp2040_hal::gpio::DYN_FLOATING_INPUT;
//! # let mut gpio12: DynPin = pins.gpio12.into();
//! // Use one of the literal function names
//! gpio12.into_floating_input();
//! // Use a method and a DynPinMode variant
//! gpio12.try_into_mode(DYN_FLOATING_INPUT).unwrap();
//! ```
//!
//! Because the pin state cannot be tracked at compile-time, many [`DynPin`]
//! operations become fallible. Run-time checks are inserted to ensure that
//! users don't try to, for example, set the output level of an input pin.
//!
//! Users may try to convert value-level pins back to their type-level
//! equivalents. However, this option is fallible, because the compiler cannot
//! guarantee the pin has the correct ID or is in the correct mode at
//! compile-time. Use [`TryFrom`](core::convert::TryFrom)/
//! [`TryInto`](core::convert::TryInto) for this conversion.
//!
//! ```no_run
//! # use core::convert::TryInto;
//! # use rp2040_hal::{pac, gpio::{DynPin, bank0::Gpio12, Pin, Pins, FloatingInput}, Sio};
//! # let mut peripherals = pac::Peripherals::take().unwrap();
//! # let sio = Sio::new(peripherals.SIO);
//! # let pins = Pins::new(peripherals.IO_BANK0,peripherals.PADS_BANK0,sio.gpio_bank0, &mut peripherals.RESETS);
//! // Convert to a `DynPin`
//! let mut gpio12: DynPin = pins.gpio12.into();
//! // Change pin mode
//! gpio12.into_floating_input();
//! // Convert back to a `Pin`
//! let gpio12: Pin<Gpio12, FloatingInput> = gpio12.try_into().unwrap();
//! ```
//!
//! # Embedded HAL traits
//!
//! This module implements all of the embedded HAL GPIO traits for [`DynPin`].
//! However, whereas the type-level API uses
//! `Error = core::convert::Infallible`, the value-level API can return a real
//! error. If the [`DynPin`] is not in the correct [`DynPinMode`] for the
//! operation, the trait functions will return
//! [`InvalidPinType`](Error::InvalidPinType).
use super::pin::{Pin, PinId, PinMode, ValidPinMode};
use super::reg::RegisterInterface;
use core::convert::TryFrom;

#[cfg(feature = "eh1_0_alpha")]
use eh1_0_alpha::digital as eh1;
use hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};

//==============================================================================
//  DynPinMode configurations
//==============================================================================

/// Value-level `enum` for disabled configurations
#[derive(PartialEq, Eq, Clone, Copy)]
#[allow(missing_docs)]
pub enum DynDisabled {
    Floating,
    PullDown,
    PullUp,
    BusKeep,
}

/// Value-level `enum` for input configurations
#[derive(PartialEq, Eq, Clone, Copy)]
#[allow(missing_docs)]
pub enum DynInput {
    Floating,
    PullDown,
    PullUp,
    BusKeep,
}

/// Value-level `enum` for output configurations
#[derive(PartialEq, Eq, Clone, Copy)]
#[allow(missing_docs)]
pub enum DynOutput {
    PushPull,
    Readable,
}

/// Value-level `enum` for output configurations
#[derive(PartialEq, Eq, Clone, Copy)]
#[allow(missing_docs)]
pub enum DynFunction {
    Spi,
    Xip,
    Uart,
    I2C,
    Pwm,
    Pio0,
    Pio1,
    Clock,
    UsbAux,
}

//==============================================================================
//  DynPinMode
//==============================================================================

/// Value-level `enum` representing pin modes
#[derive(PartialEq, Eq, Clone, Copy)]
#[allow(missing_docs)]
pub enum DynPinMode {
    Disabled(DynDisabled),
    Input(DynInput),
    Output(DynOutput),
    Function(DynFunction),
}

impl DynPinMode {
    #[inline]
    fn valid_for(&self, id: DynPinId) -> bool {
        use DynFunction::*;
        use DynGroup::*;
        use DynPinMode::*;
        match self {
            Disabled(_) => true,
            Input(_) => true,
            Output(_) => true,
            Function(alt) => match id.group {
                Bank0 => match alt {
                    Spi | Uart | I2C | Pwm | Pio0 | Pio1 | UsbAux => true,
                    Clock if id.num >= 20 && id.num <= 25 => true,
                    _ => false,
                },
                #[allow(clippy::match_like_matches_macro)]
                Qspi => match alt {
                    Xip => true,
                    _ => false,
                },
            },
        }
    }
}

/// Value-level variant of [`DynPinMode`] for floating disabled mode
pub const DYN_FLOATING_DISABLED: DynPinMode = DynPinMode::Disabled(DynDisabled::Floating);
/// Value-level variant of [`DynPinMode`] for pull-down disabled mode
pub const DYN_PULL_DOWN_DISABLED: DynPinMode = DynPinMode::Disabled(DynDisabled::PullDown);
/// Value-level variant of [`DynPinMode`] for pull-up disabled mode
pub const DYN_PULL_UP_DISABLED: DynPinMode = DynPinMode::Disabled(DynDisabled::PullUp);

/// Value-level variant of [`DynPinMode`] for floating input mode
pub const DYN_FLOATING_INPUT: DynPinMode = DynPinMode::Input(DynInput::Floating);
/// Value-level variant of [`DynPinMode`] for pull-down input mode
pub const DYN_PULL_DOWN_INPUT: DynPinMode = DynPinMode::Input(DynInput::PullDown);
/// Value-level variant of [`DynPinMode`] for pull-up input mode
pub const DYN_PULL_UP_INPUT: DynPinMode = DynPinMode::Input(DynInput::PullUp);

/// Value-level variant of [`DynPinMode`] for push-pull output mode
pub const DYN_PUSH_PULL_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::PushPull);
/// Value-level variant of [`DynPinMode`] for readable push-pull output mode
pub const DYN_READABLE_OUTPUT: DynPinMode = DynPinMode::Output(DynOutput::Readable);

macro_rules! dyn_function {
    ( $($Func:ident),+ ) => {
        crate::paste::paste! {
            $(
                #[
                    doc = "Value-level variant of [`DynPinMode`] for alternate "
                    "peripheral function " $Func
                ]
                pub const [<DYN_FUNCTION_ $Func:upper>]: DynPinMode =
                DynPinMode::Function(DynFunction::$Func);
            )+
        }
    };
}

dyn_function!(Spi, Xip, Uart, I2C, Pwm, Pio0, Pio1, Clock, UsbAux);

//==============================================================================
//  DynGroup & DynPinId
//==============================================================================

/// Value-level `enum` for pin groups
#[derive(PartialEq, Clone, Copy)]
pub enum DynGroup {
    /// .
    Bank0,
    /// .
    Qspi,
}

/// Value-level `struct` representing pin IDs
#[derive(PartialEq, Clone, Copy)]
pub struct DynPinId {
    /// .
    pub group: DynGroup,
    /// .
    pub num: u8,
}

//==============================================================================
//  DynRegisters
//==============================================================================

/// Provide a safe register interface for [`DynPin`]s
///
/// This `struct` takes ownership of a [`DynPinId`] and provides an API to
/// access the corresponding regsiters.
struct DynRegisters {
    id: DynPinId,
}

// [`DynRegisters`] takes ownership of the [`DynPinId`], and [`DynPin`]
// guarantees that each pin is a singleton, so this implementation is safe.
unsafe impl RegisterInterface for DynRegisters {
    #[inline]
    fn id(&self) -> DynPinId {
        self.id
    }
}

impl DynRegisters {
    /// Create a new instance of [`DynRegisters`]
    ///
    /// # Safety
    ///
    /// Users must never create two simultaneous instances of this `struct` with
    /// the same [`DynPinId`]
    #[inline]
    unsafe fn new(id: DynPinId) -> Self {
        DynRegisters { id }
    }
}

//==============================================================================
//  Error
//==============================================================================

/// GPIO error type
///
/// [`DynPin`]s are not tracked and verified at compile-time, so run-time
/// operations are fallible. This `enum` represents the corresponding errors.
#[derive(Debug)]
pub enum Error {
    /// The pin did not have the correct ID or mode for the requested operation
    InvalidPinType,
    /// The pin does not support the requeted mode
    InvalidPinMode,
}

//==============================================================================
//  DynPin
//==============================================================================

/// A value-level pin, parameterized by [`DynPinId`] and [`DynPinMode`]
///
/// This type acts as a type-erased version of [`Pin`]. Every pin is represented
/// by the same type, and pins are tracked and distinguished at run-time.
pub struct DynPin {
    regs: DynRegisters,
    mode: DynPinMode,
}

impl DynPin {
    /// Create a new [`DynPin`]
    ///
    /// # Safety
    ///
    /// Each [`DynPin`] must be a singleton. For a given [`DynPinId`], there
    /// must be at most one corresponding [`DynPin`] in existence at any given
    /// time.  Violating this requirement is `unsafe`.
    #[inline]
    unsafe fn new(id: DynPinId, mode: DynPinMode) -> Self {
        DynPin {
            regs: DynRegisters::new(id),
            mode,
        }
    }

    /// Return a copy of the pin ID
    #[inline]
    pub fn id(&self) -> DynPinId {
        self.regs.id
    }

    /// Return a copy of the pin mode
    #[inline]
    pub fn mode(&self) -> DynPinMode {
        self.mode
    }

    /// Convert the pin to the requested [`DynPinMode`]
    #[inline]
    pub fn try_into_mode(&mut self, mode: DynPinMode) -> Result<(), Error> {
        // FIXME: check valid modes
        // Only modify registers if we are actually changing pin mode
        if mode.valid_for(self.regs.id) {
            if mode != self.mode {
                self.regs.do_change_mode(mode);
                self.mode = mode;
            }
            Ok(())
        } else {
            Err(Error::InvalidPinMode)
        }
    }

    /// Disable the pin and set it to float
    #[inline]
    #[allow(clippy::wrong_self_convention)] // matches pin api
    pub fn into_floating_disabled(&mut self) {
        self.try_into_mode(DYN_FLOATING_DISABLED).unwrap(); // always valid
    }

    /// Disable the pin and set it to pull down
    #[inline]
    #[allow(clippy::wrong_self_convention)] // matches pin api
    pub fn into_pull_down_disabled(&mut self) {
        self.try_into_mode(DYN_PULL_DOWN_DISABLED).unwrap(); // always valid
    }

    /// Disable the pin and set it to pull up
    #[inline]
    #[allow(clippy::wrong_self_convention)] // matches pin api
    pub fn into_pull_up_disabled(&mut self) {
        self.try_into_mode(DYN_PULL_UP_DISABLED).unwrap(); // always valid
    }

    /// Configure the pin to operate as a floating input
    #[inline]
    #[allow(clippy::wrong_self_convention)] // matches pin api
    pub fn into_floating_input(&mut self) {
        self.try_into_mode(DYN_FLOATING_INPUT).unwrap(); // always valid
    }

    /// Configure the pin to operate as a pulled down input
    #[inline]
    #[allow(clippy::wrong_self_convention)] // matches pin api
    pub fn into_pull_down_input(&mut self) {
        self.try_into_mode(DYN_PULL_DOWN_INPUT).unwrap(); // always valid
    }

    /// Configure the pin to operate as a pulled up input
    #[inline]
    #[allow(clippy::wrong_self_convention)] // matches pin api
    pub fn into_pull_up_input(&mut self) {
        self.try_into_mode(DYN_PULL_UP_INPUT).unwrap(); // always valid
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    #[allow(clippy::wrong_self_convention)] // matches pin api
    pub fn into_push_pull_output(&mut self) {
        self.try_into_mode(DYN_PUSH_PULL_OUTPUT).unwrap(); // always valid
    }

    /// Configure the pin to operate as a readable push pull output
    #[inline]
    #[allow(clippy::wrong_self_convention)] // matches pin api
    pub fn into_readable_output(&mut self) {
        self.try_into_mode(DYN_READABLE_OUTPUT).unwrap(); // always valid
    }

    #[inline]
    fn _read(&self) -> Result<bool, Error> {
        match self.mode {
            DynPinMode::Input(_) | DYN_READABLE_OUTPUT => Ok(self.regs.read_pin()),
            _ => Err(Error::InvalidPinType),
        }
    }
    #[inline]
    fn _write(&mut self, bit: bool) -> Result<(), Error> {
        match self.mode {
            DynPinMode::Output(_) => {
                self.regs.write_pin(bit);
                Ok(())
            }
            _ => Err(Error::InvalidPinType),
        }
    }
    #[inline]
    fn _toggle(&mut self) -> Result<(), Error> {
        match self.mode {
            DynPinMode::Output(_) => {
                self.regs.toggle_pin();
                Ok(())
            }
            _ => Err(Error::InvalidPinType),
        }
    }
    #[inline]
    fn _read_out(&self) -> Result<bool, Error> {
        match self.mode {
            DynPinMode::Output(_) => Ok(self.regs.read_out_pin()),
            _ => Err(Error::InvalidPinType),
        }
    }
    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    fn _is_low(&self) -> Result<bool, Error> {
        Ok(self._read()? == false)
    }
    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    fn _is_high(&self) -> Result<bool, Error> {
        Ok(self._read()? == true)
    }
    #[inline]
    fn _set_low(&mut self) -> Result<(), Error> {
        self._write(false)
    }
    #[inline]
    fn _set_high(&mut self) -> Result<(), Error> {
        self._write(true)
    }
    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    fn _is_set_low(&self) -> Result<bool, Error> {
        Ok(self._read_out()? == false)
    }
    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    fn _is_set_high(&self) -> Result<bool, Error> {
        Ok(self._read_out()? == true)
    }
}

//==============================================================================
//  Convert between Pin and DynPin
//==============================================================================

impl<I, M> From<Pin<I, M>> for DynPin
where
    I: PinId,
    M: PinMode + ValidPinMode<I>,
{
    /// Erase the type-level information in a [`Pin`] and return a value-level
    /// [`DynPin`]
    #[inline]
    fn from(_pin: Pin<I, M>) -> Self {
        // The `Pin` is consumed, so it is safe to replace it with the
        // corresponding `DynPin`
        unsafe { DynPin::new(I::DYN, M::DYN) }
    }
}

impl<I, M> TryFrom<DynPin> for Pin<I, M>
where
    I: PinId,
    M: PinMode + ValidPinMode<I>,
{
    type Error = Error;

    /// Try to recreate a type-level [`Pin`] from a value-level [`DynPin`]
    ///
    /// There is no way for the compiler to know if the conversion will be
    /// successful at compile-time. We must verify the conversion at run-time
    /// or refuse to perform it.
    #[inline]
    fn try_from(pin: DynPin) -> Result<Self, Error> {
        if pin.regs.id == I::DYN && pin.mode == M::DYN {
            // The `DynPin` is consumed, so it is safe to replace it with the
            // corresponding `Pin`
            Ok(unsafe { Self::new() })
        } else {
            Err(Error::InvalidPinType)
        }
    }
}

//==============================================================================
// Embedded HAL traits
//==============================================================================

impl OutputPin for DynPin {
    type Error = Error;
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self._set_high()
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self._set_low()
    }
}

impl InputPin for DynPin {
    type Error = Error;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        self._is_high()
    }
    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        self._is_low()
    }
}

impl ToggleableOutputPin for DynPin {
    type Error = Error;
    #[inline]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self._toggle()
    }
}

impl StatefulOutputPin for DynPin {
    #[inline]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        self._is_set_high()
    }
    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        self._is_set_low()
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl eh1::ErrorType for DynPin {
    type Error = Error;
}

#[cfg(feature = "eh1_0_alpha")]
impl eh1::blocking::OutputPin for DynPin {
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self._set_high()
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self._set_low()
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl eh1::blocking::InputPin for DynPin {
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        self._is_high()
    }
    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        self._is_low()
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl eh1::blocking::ToggleableOutputPin for DynPin {
    #[inline]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self._toggle()
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl eh1::blocking::StatefulOutputPin for DynPin {
    #[inline]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        self._is_set_high()
    }
    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        self._is_set_low()
    }
}
