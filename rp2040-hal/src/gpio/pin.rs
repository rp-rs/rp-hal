//! # Type-level module for GPIO pins
//!
//! Based heavily on `atsamd-hal`.
//!
//! This module provides a type-level API for GPIO pins. It uses the type system
//! to track the state of pins at compile-time. To do so, it uses traits to
//! represent [type classes] and types as instances of those type classes. For
//! example, the trait [`InputConfig`] acts as a [type-level enum] of the
//! available input configurations, and the types [`Floating`], [`PullDown`],
//! [`PullUp`] and [`BusKeep`] are its type-level variants.
//!
//! When applied as a trait bound, a type-level enum restricts type parameters
//! to the corresponding variants. All of the traits in this module are closed,
//! using the `Sealed` trait pattern, so the type-level instances found in this
//! module are the only possible variants.
//!
//! Type-level [`Pin`]s are parameterized by two type-level enums, [`PinId`] and
//! [`PinMode`].
//!
//! A `PinId` identifies a pin by it's group (BANK0 or QSPI) and pin number. Each
//! `PinId` instance is named according to its datasheet identifier, e.g.
//! [`Gpio0`](`bank0::Gpio0`).
//!
//! A `PinMode` represents the various pin modes. The available `PinMode`
//! variants are [`Disabled`], [`Input`], [`Output`] and
//! [`Function`], each with its own corresponding configurations.
//!
//! It is not possible for users to create new instances of a [`Pin`]. Singleton
//! instances of each pin are made available to users through the [`Pins`]
//! struct.
//!
//! To create the [`Pins`] struct, users must supply the PAC
//! [`IO_BANK0`](crate::pac::IO_BANK0) and [`PAD_BANK0`](crate::pac::PADS_BANK0) peripherals as well as the [SIO partition](crate::sio).
//! The [`Pins`] struct takes
//! ownership of the peripherals and provides the corresponding pins. Each [`Pin`]
//! within the [`Pins`] struct can be moved out and used individually.
//!
//!
//! ```no_run
//! # use rp2040_hal::{pac, gpio::Pins, sio::Sio};
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0,peripherals.PADS_BANK0,sio.gpio_bank0, &mut peripherals.RESETS);
//! ```
//!
//! Pins can be converted between modes using several different methods.
//!
//! ```no_run
//! # use rp2040_hal::{pac, gpio::{bank0::Gpio12, Pin, Pins, FloatingInput}, sio::Sio};
//! # let mut peripherals = pac::Peripherals::take().unwrap();
//! # let sio = Sio::new(peripherals.SIO);
//! # let pins = Pins::new(peripherals.IO_BANK0,peripherals.PADS_BANK0,sio.gpio_bank0, &mut peripherals.RESETS);
//! // Use one of the literal function names
//! let gpio12 = pins.gpio12.into_floating_input();
//! // Use a generic method and one of the `PinMode` variant types
//! let gpio12 = gpio12.into_mode::<FloatingInput>();
//! // Specify the target type and use `.into_mode()`
//! let gpio12: Pin<Gpio12, FloatingInput> = gpio12.into_mode();
//! ```
//!
//! # Embedded HAL traits
//!
//! This module implements all of the embedded HAL GPIO traits for each [`Pin`]
//! in the corresponding [`PinMode`]s, namely: [`InputPin`], [`OutputPin`],
//! [`ToggleableOutputPin`] and [`StatefulOutputPin`].
//!
//! For example, you can control the logic level of an `OutputPin` like so
//!
//! ```no_run
//! use rp2040_hal::{pac, gpio::{bank0::Gpio12, Pin, Pins, PushPullOutput}, sio::Sio};
//! use embedded_hal::digital::v2::OutputPin;
//!
//! let mut peripherals = pac::Peripherals::take().unwrap();
//! let sio = Sio::new(peripherals.SIO);
//! let pins = Pins::new(peripherals.IO_BANK0,peripherals.PADS_BANK0,sio.gpio_bank0, &mut peripherals.RESETS);
//!
//! let mut pin12: Pin<Gpio12, PushPullOutput> = pins.gpio12.into_mode();
//! pin12.set_high();
//! ```
//!
//! # Type-level features
//!
//! This module also provides additional, type-level tools to work with GPIO
//! pins.
//!
//! The [`OptionalPinId`] and [`OptionalPin`] traits use the [`OptionalKind`]
//! pattern to act as type-level versions of [`Option`] for `PinId` and `Pin`
//! respectively. And the [`AnyPin`] trait defines an [`AnyKind`] type class
//! for all `Pin` types.
//!
//! [type classes]: crate::typelevel#type-classes
//! [type-level enum]: crate::typelevel#type-level-enum
//! [`OptionalKind`]: crate::typelevel#optionalkind-trait-pattern
//! [`AnyKind`]: crate::typelevel#anykind-trait-pattern
use super::dynpin::{DynDisabled, DynInput, DynOutput, DynPinId, DynPinMode};
use super::{
    InputOverride, Interrupt, InterruptOverride, OutputDriveStrength, OutputEnableOverride,
    OutputOverride, OutputSlewRate,
};
use crate::gpio::reg::RegisterInterface;
use crate::typelevel::{Is, NoneT, Sealed};
use core::convert::Infallible;
use core::marker::PhantomData;

use crate::gpio::dynpin::DynFunction;
#[cfg(feature = "eh1_0_alpha")]
use eh1_0_alpha::digital as eh1;
use hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin, ToggleableOutputPin};

use core::mem::transmute;

/// Type-level marker for tracking which pin modes are valid for which pins
pub trait ValidPinMode<I: PinId>: Sealed {}

//==============================================================================
//  Disabled configurations
//==============================================================================

/// Type-level `enum` for disabled configurations
pub trait DisabledConfig: Sealed {
    /// Corresponding [`DynDisabled`](super::DynDisabled)
    const DYN: DynDisabled;
}

/// Type-level variant of both [`DisabledConfig`] and [`InputConfig`]
pub enum Floating {}
/// Type-level variant of both [`DisabledConfig`] and [`InputConfig`]
pub enum PullDown {}
/// Type-level variant of both [`DisabledConfig`] and [`InputConfig`]
pub enum PullUp {}
/// Type-level variant of both [`DisabledConfig`] and [`InputConfig`]
pub enum BusKeep {}

impl Sealed for Floating {}
impl Sealed for PullDown {}
impl Sealed for PullUp {}
impl Sealed for BusKeep {}

impl DisabledConfig for Floating {
    const DYN: DynDisabled = DynDisabled::Floating;
}
impl DisabledConfig for PullDown {
    const DYN: DynDisabled = DynDisabled::PullDown;
}
impl DisabledConfig for PullUp {
    const DYN: DynDisabled = DynDisabled::PullUp;
}
impl DisabledConfig for BusKeep {
    const DYN: DynDisabled = DynDisabled::BusKeep;
}

/// Type-level variant of [`PinMode`] for disabled modes
///
/// Type `C` is one of four configurations: [`Floating`], [`PullDown`],
/// [`PullUp`] or [`BusKeep`]
pub struct Disabled<C: DisabledConfig> {
    cfg: PhantomData<C>,
}

impl<C: DisabledConfig> Sealed for Disabled<C> {}

/// Type-level variant of [`PinMode`] for floating disabled mode
pub type FloatingDisabled = Disabled<Floating>;

/// Type-level variant of [`PinMode`] for pull-down disabled mode
pub type PullDownDisabled = Disabled<PullDown>;

/// Type-level variant of [`PinMode`] for pull-up disabled mode
pub type PullUpDisabled = Disabled<PullUp>;

/// Type-level variant of [`PinMode`] for bus keep disabled mode
pub type BusKeepDisabled = Disabled<BusKeep>;

impl<I: PinId, C: DisabledConfig> ValidPinMode<I> for Disabled<C> {}

//==============================================================================
//  Input configurations
//==============================================================================

/// Type-level `enum` for input configurations
pub trait InputConfig: Sealed {
    /// Corresponding [`DynInput`](super::DynInput)
    const DYN: DynInput;
}

impl InputConfig for Floating {
    const DYN: DynInput = DynInput::Floating;
}
impl InputConfig for PullDown {
    const DYN: DynInput = DynInput::PullDown;
}
impl InputConfig for PullUp {
    const DYN: DynInput = DynInput::PullUp;
}
impl InputConfig for BusKeep {
    const DYN: DynInput = DynInput::BusKeep;
}

/// Type-level variant of [`PinMode`] for input modes
///
/// Type `C` is one of four input configurations: [`Floating`], [`PullDown`],
/// [`PullUp`] or [`BusKeep`]
pub struct Input<C: InputConfig> {
    cfg: PhantomData<C>,
}

impl<C: InputConfig> Sealed for Input<C> {}

/// Type-level variant of [`PinMode`] for floating input mode
pub type FloatingInput = Input<Floating>;

/// Type-level variant of [`PinMode`] for pull-down input mode
pub type PullDownInput = Input<PullDown>;

/// Type-level variant of [`PinMode`] for pull-up input mode
pub type PullUpInput = Input<PullUp>;

/// Type-level variant of [`PinMode`] for bus keep input mode
pub type BusKeepInput = Input<BusKeep>;

impl<I: PinId, C: InputConfig> ValidPinMode<I> for Input<C> {}

//==============================================================================
//  Output configurations
//==============================================================================

/// Type-level `enum` for output configurations
pub trait OutputConfig: Sealed {
    /// Corresponding [`DynOutput`](super::DynOutput)
    const DYN: DynOutput;
}

/// Type-level variant of [`OutputConfig`] for a push-pull configuration
pub enum PushPull {}
/// Type-level variant of [`OutputConfig`] for a readable push-pull
/// configuration
pub enum Readable {}

impl Sealed for PushPull {}
impl Sealed for Readable {}

impl OutputConfig for PushPull {
    const DYN: DynOutput = DynOutput::PushPull;
}
impl OutputConfig for Readable {
    const DYN: DynOutput = DynOutput::Readable;
}

/// Type-level variant of [`PinMode`] for output modes
///
/// Type `C` is one of two output configurations: [`PushPull`] or [`Readable`]
pub struct Output<C: OutputConfig> {
    cfg: PhantomData<C>,
}

impl<C: OutputConfig> Sealed for Output<C> {}

/// Type-level variant of [`PinMode`] for push-pull output mode
pub type PushPullOutput = Output<PushPull>;

/// Type-level variant of [`PinMode`] for readable push-pull output mode
pub type ReadableOutput = Output<Readable>;

impl<I: PinId, C: OutputConfig> ValidPinMode<I> for Output<C> {}

//

/// Type-level variant of [`PinMode`] for alternate peripheral functions
///
/// Type `C` is an [`FunctionConfig`]
pub struct Function<C: FunctionConfig> {
    cfg: PhantomData<C>,
}

impl<C: FunctionConfig> Sealed for Function<C> {}

/// Type-level enum for alternate peripheral function configurations
pub trait FunctionConfig: Sealed {
    /// Corresponding [`DynFunction`](super::DynFunction)
    const DYN: DynFunction;
}

macro_rules! function {
    (
        $(
            $Func:ident
        ),+
    ) => {
        $crate::paste::paste! {
            $(
                #[
                    doc = "Type-level variant of [`FunctionConfig`] for \
                    alternate peripheral function " $Func
                ]
                pub enum $Func {}
                impl Sealed for $Func {}
                impl FunctionConfig for $Func {
                    const DYN: DynFunction = DynFunction::$Func;
                }
                #[
                    doc = "Type-level variant of [`PinMode`] for alternate \
                    peripheral function [`" $Func "`]"
                ]
                pub type [<Function $Func>] = Function<$Func>;
            )+
        }
    };
}

function!(Spi, Xip, Uart, I2C, Pwm, Clock, UsbAux);

impl Sealed for pac::PIO0 {}
impl FunctionConfig for pac::PIO0 {
    const DYN: DynFunction = DynFunction::Pio0;
}
/// Type-level variant of [`PinMode`] for alternate peripheral function `pac::PIO0`
pub type FunctionPio0 = Function<pac::PIO0>;

impl Sealed for pac::PIO1 {}
impl FunctionConfig for pac::PIO1 {
    const DYN: DynFunction = DynFunction::Pio1;
}
/// Type-level variant of [`PinMode`] for alternate peripheral function `pac::PIO1`
pub type FunctionPio1 = Function<pac::PIO1>;

//==============================================================================
//  Pin modes
//==============================================================================

/// Type-level `enum` representing pin modes
pub trait PinMode: Sealed + Sized {
    /// Corresponding [`DynPinMode`](super::DynPinMode)
    const DYN: DynPinMode;
}

impl<C: DisabledConfig> PinMode for Disabled<C> {
    const DYN: DynPinMode = DynPinMode::Disabled(C::DYN);
}

impl<C: InputConfig> PinMode for Input<C> {
    const DYN: DynPinMode = DynPinMode::Input(C::DYN);
}

impl<C: OutputConfig> PinMode for Output<C> {
    const DYN: DynPinMode = DynPinMode::Output(C::DYN);
}

impl<C: FunctionConfig> PinMode for Function<C> {
    const DYN: DynPinMode = DynPinMode::Function(C::DYN);
}

//==============================================================================
//  Pin IDs
//==============================================================================

/// Type-level `enum` for pin IDs
pub trait PinId: Sealed {
    /// Corresponding [`DynPinId`](super::DynPinId)
    const DYN: DynPinId;
    /// [`PinMode`] at reset
    type Reset;
}

macro_rules! pin_id {
    ($Group:ident, $Id:ident, $NUM:literal, $reset : ident) => {
        #[doc = "Pin ID representing pin "]
        pub enum $Id {}
        impl Sealed for $Id {}
        impl PinId for $Id {
            type Reset = $reset;
            const DYN: DynPinId = DynPinId {
                group: DynGroup::$Group,
                num: $NUM,
            };
        }
    };
}

//==============================================================================
//  OptionalPinId
//==============================================================================

/// Type-level equivalent of `Option<PinId>`
///
/// See the [`OptionalKind`] documentation for more details on the pattern.
///
/// [`OptionalKind`]: crate::typelevel#optionalkind-trait-pattern
pub trait OptionalPinId {}

impl OptionalPinId for NoneT {}

impl<I: PinId> OptionalPinId for I {}

/// Type-level equivalent of `Some(PinId)`
///
/// See the [`OptionalKind`] documentation for more details on the pattern.
///
/// [`OptionalKind`]: crate::typelevel#optionalkind-trait-pattern
pub trait SomePinId: OptionalPinId + PinId {}

impl<I: PinId> SomePinId for I {}

//==============================================================================
//  Registers
//==============================================================================

/// Provide a safe register interface for [`Pin`]s
///
/// This `struct` takes ownership of a [`PinId`] and provides an API to
/// access the corresponding regsiters.
struct Registers<I: PinId> {
    id: PhantomData<I>,
}

// [`Registers`] takes ownership of the [`PinId`], and [`Pin`] guarantees that
// each pin is a singleton, so this implementation is safe.
unsafe impl<I: PinId> RegisterInterface for Registers<I> {
    #[inline]
    fn id(&self) -> DynPinId {
        I::DYN
    }
}

impl<I: PinId> Registers<I> {
    /// Create a new instance of [`Registers`]
    ///
    /// # Safety
    ///
    /// Users must never create two simultaneous instances of this `struct` with
    /// the same [`PinId`]
    #[inline]
    unsafe fn new() -> Self {
        Registers { id: PhantomData }
    }

    /// Provide a type-level equivalent for the
    /// [`RegisterInterface::change_mode`] method.
    #[inline]
    fn change_mode<M: PinMode + ValidPinMode<I>>(&mut self) {
        RegisterInterface::do_change_mode(self, M::DYN);
    }
}

//==============================================================================
//  Pin
//==============================================================================

/// A type-level GPIO pin, parameterized by [`PinId`] and [`PinMode`] types
pub struct Pin<I, M>
where
    I: PinId,
    M: PinMode + ValidPinMode<I>,
{
    regs: Registers<I>,
    mode: PhantomData<M>,
}

impl<I, M> Pin<I, M>
where
    I: PinId,
    M: PinMode + ValidPinMode<I>,
{
    /// Create a new [`Pin`]
    ///
    /// # Safety
    ///
    /// Each [`Pin`] must be a singleton. For a given [`PinId`], there must be
    /// at most one corresponding [`Pin`] in existence at any given time.
    /// Violating this requirement is `unsafe`.
    #[inline]
    pub(crate) unsafe fn new() -> Pin<I, M> {
        Pin {
            regs: Registers::new(),
            mode: PhantomData,
        }
    }

    /// Convert the pin to the requested [`PinMode`]
    #[inline]
    pub fn into_mode<N: PinMode + ValidPinMode<I>>(mut self) -> Pin<I, N> {
        if N::DYN != M::DYN {
            self.regs.change_mode::<N>();
        }
        // Safe because we drop the existing Pin
        unsafe { Pin::new() }
    }

    /// Disable the pin and set it to float
    #[inline]
    pub fn into_floating_disabled(self) -> Pin<I, FloatingDisabled> {
        self.into_mode()
    }

    /// Disable the pin and set it to pull down
    #[inline]
    pub fn into_pull_down_disabled(self) -> Pin<I, PullDownDisabled> {
        self.into_mode()
    }

    /// Disable the pin and set it to pull up
    #[inline]
    pub fn into_pull_up_disabled(self) -> Pin<I, PullUpDisabled> {
        self.into_mode()
    }

    /// Configure the pin to operate as a floating input
    #[inline]
    pub fn into_floating_input(self) -> Pin<I, FloatingInput> {
        self.into_mode()
    }

    /// Configure the pin to operate as a pulled down input
    #[inline]
    pub fn into_pull_down_input(self) -> Pin<I, PullDownInput> {
        self.into_mode()
    }

    /// Configure the pin to operate as a pulled up input
    #[inline]
    pub fn into_pull_up_input(self) -> Pin<I, PullUpInput> {
        self.into_mode()
    }

    /// Configure the pin to operate as a bus keep input
    #[inline]
    pub fn into_bus_keep_input(self) -> Pin<I, BusKeepInput> {
        self.into_mode()
    }

    /// Configure the pin to operate as a push-pull output
    #[inline]
    pub fn into_push_pull_output(self) -> Pin<I, PushPullOutput> {
        self.into_mode()
    }

    /// Configure the pin to operate as a readable push pull output
    #[inline]
    pub fn into_readable_output(self) -> Pin<I, ReadableOutput> {
        self.into_mode()
    }

    /// Read the current drive strength of the pin.
    #[inline]
    pub fn get_drive_strength(&self) -> OutputDriveStrength {
        self.regs.read_drive_strength()
    }

    /// Set the drive strength for the pin.
    #[inline]
    pub fn set_drive_strength(&mut self, strength: OutputDriveStrength) {
        self.regs.write_drive_strength(strength);
    }

    /// Get the slew rate for the pin.
    #[inline]
    pub fn get_slew_rate(&self) -> OutputSlewRate {
        self.regs.read_slew_rate()
    }

    /// Set the slew rate for the pin.
    #[inline]
    pub fn set_slew_rate(&mut self, rate: OutputSlewRate) {
        self.regs.write_slew_rate(rate)
    }

    /// Clear interrupt.
    #[inline]
    pub fn clear_interrupt(&mut self, interrupt: Interrupt) {
        self.regs.clear_interrupt(interrupt);
    }

    /// Interrupt status.
    #[inline]
    pub fn interrupt_status(&self, interrupt: Interrupt) -> bool {
        self.regs.interrupt_status(interrupt)
    }

    /// Is interrupt enabled.
    #[inline]
    pub fn is_interrupt_enabled(&self, interrupt: Interrupt) -> bool {
        self.regs.is_interrupt_enabled(interrupt)
    }

    /// Enable or disable interrupt.
    #[inline]
    pub fn set_interrupt_enabled(&self, interrupt: Interrupt, enabled: bool) {
        self.regs.set_interrupt_enabled(interrupt, enabled);
    }

    /// Is interrupt forced.
    #[inline]
    pub fn is_interrupt_forced(&self, interrupt: Interrupt) -> bool {
        self.regs.is_interrupt_forced(interrupt)
    }

    /// Force or release interrupt.
    #[inline]
    pub fn set_interrupt_forced(&self, interrupt: Interrupt, forced: bool) {
        self.regs.set_interrupt_forced(interrupt, forced);
    }

    /// Set the interrupt override.
    #[inline]
    pub fn set_interrupt_override(&mut self, override_value: InterruptOverride) {
        self.regs.set_interrupt_override(override_value);
    }

    /// Set the input override.
    #[inline]
    pub fn set_input_override(&mut self, override_value: InputOverride) {
        self.regs.set_input_override(override_value);
    }

    /// Set the output enable override.
    #[inline]
    pub fn set_output_enable_override(&mut self, override_value: OutputEnableOverride) {
        self.regs.set_output_enable_override(override_value);
    }

    /// Set the output override.
    #[inline]
    pub fn set_output_override(&mut self, override_value: OutputOverride) {
        self.regs.set_output_override(override_value);
    }

    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    pub(crate) fn _is_low(&self) -> bool {
        self.regs.read_pin() == false
    }

    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    pub(crate) fn _is_high(&self) -> bool {
        self.regs.read_pin() == true
    }

    #[inline]
    pub(crate) fn _set_low(&mut self) {
        self.regs.write_pin(false);
    }

    #[inline]
    pub(crate) fn _set_high(&mut self) {
        self.regs.write_pin(true);
    }

    #[inline]
    pub(crate) fn _toggle(&mut self) {
        self.regs.toggle_pin();
    }

    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    pub(crate) fn _is_set_low(&self) -> bool {
        self.regs.read_out_pin() == false
    }

    #[inline]
    #[allow(clippy::bool_comparison)] // more explicit this way
    pub(crate) fn _is_set_high(&self) -> bool {
        self.regs.read_out_pin() == true
    }
}

//==============================================================================
//  AnyPin
//==============================================================================

/// Type class for [`Pin`] types
///
/// This trait uses the [`AnyKind`] trait pattern to create a [type class] for
/// [`Pin`] types. See the `AnyKind` documentation for more details on the
/// pattern.
///
/// [`AnyKind`]: crate::typelevel#anykind-trait-pattern
/// [type class]: crate::typelevel#type-classes
pub trait AnyPin
where
    Self: Sealed,
    Self: Is<Type = SpecificPin<Self>>,
    <Self as AnyPin>::Mode: ValidPinMode<<Self as AnyPin>::Id>,
{
    /// [`PinId`] of the corresponding [`Pin`]
    type Id: PinId;
    /// [`PinMode`] of the corresponding [`Pin`]
    type Mode: PinMode;
}

impl<I, M> Sealed for Pin<I, M>
where
    I: PinId,
    M: PinMode + ValidPinMode<I>,
{
}

impl<I, M> AnyPin for Pin<I, M>
where
    I: PinId,
    M: PinMode + ValidPinMode<I>,
{
    type Id = I;
    type Mode = M;
}

/// Type alias to recover the specific [`Pin`] type from an implementation of
/// [`AnyPin`]
///
/// See the [`AnyKind`] documentation for more details on the pattern.
///
/// [`AnyKind`]: crate::typelevel#anykind-trait-pattern
pub type SpecificPin<P> = Pin<<P as AnyPin>::Id, <P as AnyPin>::Mode>;

impl<P: AnyPin> AsRef<P> for SpecificPin<P> {
    #[inline]
    fn as_ref(&self) -> &P {
        // SAFETY: This is guaranteed to be safe, because P == SpecificPin<P>
        // Transmuting between `v1` and `v2` `Pin` types is also safe, because
        // both are zero-sized, and single-field, newtype structs are guaranteed
        // to have the same layout as the field anyway, even for repr(Rust).
        unsafe { transmute(self) }
    }
}

impl<P: AnyPin> AsMut<P> for SpecificPin<P> {
    #[inline]
    fn as_mut(&mut self) -> &mut P {
        // SAFETY: This is guaranteed to be safe, because P == SpecificPin<P>
        // Transmuting between `v1` and `v2` `Pin` types is also safe, because
        // both are zero-sized, and single-field, newtype structs are guaranteed
        // to have the same layout as the field anyway, ValidPinMode<P::Id> en for repr(Rust).
        unsafe { transmute(self) }
    }
}

//==============================================================================
//  Optional pins
//==============================================================================

/// Type-level equivalent of `Option<PinId>`
///
/// See the [`OptionalKind`] documentation for more details on the pattern.
///
/// [`OptionalKind`]: crate::typelevel#optionalkind-trait-pattern
pub trait OptionalPin: Sealed {
    #[allow(missing_docs)]
    type Id: OptionalPinId;
}

impl OptionalPin for NoneT {
    type Id = NoneT;
}

impl<P: AnyPin> OptionalPin for P {
    type Id = P::Id;
}

/// Type-level equivalent of `Some(PinId)`
///
/// See the [`OptionalKind`] documentation for more details on the pattern.
///
/// [`OptionalKind`]: crate::typelevel#optionalkind-trait-pattern
pub trait SomePin: AnyPin {}
impl<P: AnyPin> SomePin for P {}

//==============================================================================
//  Embedded HAL traits
//==============================================================================

impl<I, C> OutputPin for Pin<I, Output<C>>
where
    I: PinId,
    C: OutputConfig,
{
    type Error = Infallible;
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self._set_high();
        Ok(())
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self._set_low();
        Ok(())
    }
}

impl<I> InputPin for Pin<I, ReadableOutput>
where
    I: PinId,
{
    type Error = Infallible;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }
    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

impl<I, C> InputPin for Pin<I, Input<C>>
where
    I: PinId,
    C: InputConfig,
{
    type Error = Infallible;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }
    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

impl<I, C> ToggleableOutputPin for Pin<I, Output<C>>
where
    I: PinId,
    C: OutputConfig,
{
    type Error = Infallible;
    #[inline]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self._toggle();
        Ok(())
    }
}

impl<I, C> StatefulOutputPin for Pin<I, Output<C>>
where
    I: PinId,
    C: OutputConfig,
{
    #[inline]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_set_high())
    }
    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_set_low())
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<I, C> eh1::ErrorType for Pin<I, Output<C>>
where
    I: PinId,
    C: OutputConfig,
{
    type Error = Infallible;
}

#[cfg(feature = "eh1_0_alpha")]
impl<I, C> eh1::blocking::OutputPin for Pin<I, Output<C>>
where
    I: PinId,
    C: OutputConfig,
{
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self._set_high();
        Ok(())
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self._set_low();
        Ok(())
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<I> eh1::blocking::InputPin for Pin<I, ReadableOutput>
where
    I: PinId,
{
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }
    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<I, C> eh1::ErrorType for Pin<I, Input<C>>
where
    I: PinId,
    C: InputConfig,
{
    type Error = Infallible;
}

#[cfg(feature = "eh1_0_alpha")]
impl<I, C> eh1::blocking::InputPin for Pin<I, Input<C>>
where
    I: PinId,
    C: InputConfig,
{
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_high())
    }
    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_low())
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<I, C> eh1::blocking::ToggleableOutputPin for Pin<I, Output<C>>
where
    I: PinId,
    C: OutputConfig,
{
    #[inline]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        self._toggle();
        Ok(())
    }
}

#[cfg(feature = "eh1_0_alpha")]
impl<I, C> eh1::blocking::StatefulOutputPin for Pin<I, Output<C>>
where
    I: PinId,
    C: OutputConfig,
{
    #[inline]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self._is_set_high())
    }
    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(self._is_set_low())
    }
}

//==============================================================================
//  Pin definitions
//==============================================================================

macro_rules! gpio {
    ($Group:ident, [ $($Func:ident),+ ], [
        $($PXi:ident: ($i:expr, $is:expr, $reset:ident $(, [ $($PinFunc:ident),+ ])? )),+
    ]) => {
        $crate::paste::paste! {
            #[doc = "GPIO Pins for " $Group]
            pub mod [<$Group:lower>] {
                use crate::sio::[<SioGpio $Group>];
                use pac::{[<IO_ $Group:upper>],[<PADS_ $Group:upper>]};

                /// Bank0 bank pin id
                pub trait BankPinId {}
                use crate::typelevel::Sealed;
                use crate::gpio::dynpin::{DynGroup,DynPinId};

                // FIXME: Somehow just import what we need
                #[allow(unused_imports)]
                use super::{PullDownDisabled,PullUpDisabled,FloatingDisabled,BusKeepDisabled};

                use super::{Pin,PinId};
                use crate::resets::SubsystemReset;

                $(
                    pin_id!($Group, $PXi, $i, $reset);
                    impl BankPinId for $PXi {}

                    $( $(impl super::ValidPinMode<$PXi> for super::Function<super::$PinFunc> {})+ )*
                )+

                /// Collection of all the individual [`Pin`]s
                pub struct Pins {
                    _io: [<IO_ $Group:upper>],
                    _pads: [<PADS_ $Group:upper>],
                    _sio: [<SioGpio $Group>],
                    $(
                        #[doc = "Pin " $PXi]
                        pub [<$PXi:lower>] : Pin<$PXi,<$PXi as PinId>::Reset>,
                    )+
                }

                impl Pins {
                    /// Take ownership of the PAC peripherals and SIO slice and split it into discrete [`Pin`]s
                    pub fn new(io : [<IO_ $Group:upper>], pads: [<PADS_ $Group:upper>], sio: [<SioGpio $Group>], reset : &mut pac::RESETS) -> Self {
                        pads.reset_bring_down(reset);
                        io.reset_bring_down(reset);

                        io.reset_bring_up(reset);
                        pads.reset_bring_up(reset);
                        unsafe {
                            Self {
                                _io: io,
                                _pads: pads,
                                _sio: sio,
                                $(
                                    [<$PXi:lower>]: Pin::new(),
                                )+
                            }
                        }
                    }
                }

                $( impl<I: PinId + BankPinId> super::ValidPinMode<I> for super::[<Function $Func>] {} )+
            }
        }
    }
}

gpio!(
    Bank0, [ Spi, Uart, I2C, Pwm, Pio0, Pio1, UsbAux ], [
        Gpio0: (0, "0", PullDownDisabled),
        Gpio1: (1, "1", PullDownDisabled),
        Gpio2: (2, "2", PullDownDisabled),
        Gpio3: (3, "3", PullDownDisabled),
        Gpio4: (4, "4", PullDownDisabled),
        Gpio5: (5, "5", PullDownDisabled),
        Gpio6: (6, "6", PullDownDisabled),
        Gpio7: (7, "7", PullDownDisabled),
        Gpio8: (8, "8", PullDownDisabled),
        Gpio9: (9, "9", PullDownDisabled),
        Gpio10: (10, "10", PullDownDisabled),
        Gpio11: (11, "11", PullDownDisabled),
        Gpio12: (12, "12", PullDownDisabled),
        Gpio13: (13, "13", PullDownDisabled),
        Gpio14: (14, "14", PullDownDisabled),
        Gpio15: (15, "15", PullDownDisabled),
        Gpio16: (16, "16", PullDownDisabled),
        Gpio17: (17, "17", PullDownDisabled),
        Gpio18: (18, "18", PullDownDisabled),
        Gpio19: (19, "19", PullDownDisabled),
        Gpio20: (20, "20", PullDownDisabled, [Clock]),
        Gpio21: (21, "21", PullDownDisabled, [Clock]),
        Gpio22: (22, "22", PullDownDisabled, [Clock]),
        Gpio23: (23, "23", PullDownDisabled, [Clock]),
        Gpio24: (24, "24", PullDownDisabled, [Clock]),
        Gpio25: (25, "25", PullDownDisabled, [Clock]),
        Gpio26: (26, "26", PullDownDisabled),
        Gpio27: (27, "27", PullDownDisabled),
        Gpio28: (28, "28", PullDownDisabled),
        Gpio29: (29, "29", PullDownDisabled)
    ]
);

pub use bank0::Pins; // this is probably the default everyone is going to want

gpio!(
    Qspi, [ Xip ], [
        Sck: (0, "sck", PullDownDisabled),
        Cs: (1, "cs", PullUpDisabled),
        Sd0: (2, "sd0", FloatingDisabled),
        Sd1: (3, "sd1", FloatingDisabled),
        Sd2: (4, "sd2", FloatingDisabled),
        Sd3: (5, "sd3", FloatingDisabled)
    ]
);

//==============================================================================
//  bsp_pins
//==============================================================================

/// Helper macro to give meaningful names to GPIO pins
///
/// The normal [`Pins`] struct names each [`Pin`] according to its [`PinId`].
/// However, BSP authors would prefer to name each [`Pin`] according to its
/// function. This macro defines a new `Pins` struct with custom field names
/// for each [`Pin`], and it defines type aliases and constants to make it
/// easier to work with the [`Pin`]s and [`DynPin`](super::DynPin)s.
///
/// When specifying pin aliases, be sure to use a [`PinMode`]. See
/// [here](self#types) for a list of the available [`PinMode`] type aliases.
///
/// # Example
/// ...
#[macro_export]
macro_rules! bsp_pins {
    (
        $(
            $( #[$id_cfg:meta] )*
            $Id:ident {
                $( #[$name_doc:meta] )*
                name: $name:ident $(,)?
                $(
                    aliases: {
                        $(
                            $( #[$alias_cfg:meta] )*
                            $Mode:ident: $Alias:ident
                        ),+
                    }
                )?
            } $(,)?
        )+
    ) => {
        $crate::paste::paste! {

            /// BSP replacement for the HAL
            /// [`Pins`](rp2040_hal::gpio::Pins) type
            ///
            /// This type is intended to provide more meaningful names for the
            /// given pins.
            ///
            /// To enable specific functions of the pins you can use the
            /// [rp2040_hal::gpio::pin::Pin::into_mode] function with
            /// one of:
            /// - [rp2040_hal::gpio::pin::FunctionI2C]
            /// - [rp2040_hal::gpio::pin::FunctionPwm]
            /// - [rp2040_hal::gpio::pin::FunctionSpi]
            /// - [rp2040_hal::gpio::pin::FunctionXip]
            /// - [rp2040_hal::gpio::pin::FunctionPio0]
            /// - [rp2040_hal::gpio::pin::FunctionPio1]
            /// - [rp2040_hal::gpio::pin::FunctionUart]
            ///
            /// like this:
            ///```no_run
            /// use rp2040_hal::{pac, gpio::{bank0::Gpio12, Pin, Pins, PushPullOutput}, sio::Sio};
            ///
            /// let mut peripherals = pac::Peripherals::take().unwrap();
            /// let sio = Sio::new(peripherals.SIO);
            /// let pins = Pins::new(peripherals.IO_BANK0,peripherals.PADS_BANK0,sio.gpio_bank0, &mut peripherals.RESETS);
            ///
            /// let _spi_sclk = pins.gpio2.into_mode::<rp2040_hal::gpio::FunctionSpi>();
            /// let _spi_mosi = pins.gpio3.into_mode::<rp2040_hal::gpio::FunctionSpi>();
            /// let _spi_miso = pins.gpio4.into_mode::<rp2040_hal::gpio::FunctionSpi>();
            ///```
            ///
            /// **See also [rp2040_hal::gpio::pin] for more in depth information
            /// about this**!
            pub struct Pins {
                $(
                    $( #[$id_cfg] )*
                    $( #[$name_doc] )*
                    pub $name: $crate::gpio::Pin<
                        $crate::gpio::bank0::$Id,
                        <$crate::gpio::bank0::$Id as $crate::gpio::PinId>::Reset
                    >,
                )+
            }

            impl Pins {
                /// Take ownership of the PAC [`PORT`] and split it into
                /// discrete [`Pin`]s.
                ///
                /// This struct serves as a replacement for the HAL [`Pins`]
                /// struct. It is intended to provide more meaningful names for
                /// each [`Pin`] in a BSP. Any [`Pin`] not defined by the BSP is
                /// dropped.
                ///
                /// [`Pin`](rp2040_hal::gpio::Pin)
                /// [`Pins`](rp2040_hal::gpio::Pins)
                #[inline]
                pub fn new(io : $crate::pac::IO_BANK0, pads: $crate::pac::PADS_BANK0, sio: $crate::sio::SioGpioBank0, reset : &mut $crate::pac::RESETS) -> Self {
                    let mut pins = $crate::gpio::Pins::new(io,pads,sio,reset);
                    Self {
                        $(
                            $( #[$id_cfg] )*
                            $name: pins.[<$Id:lower>],
                        )+
                    }
                }
            }
            $(
                $( #[$id_cfg] )*
                $crate::bsp_pins!(@aliases, $( $( $( #[$alias_cfg] )* $Id $Mode $Alias )+ )? );
            )+
        }
    };
    ( @aliases, $( $( $( #[$attr:meta] )* $Id:ident $Mode:ident $Alias:ident )+ )? ) => {
        $crate::paste::paste! {
            $(
                $(
                    $( #[$attr] )*
                    /// Alias for a configured [`Pin`](rp2040_hal::gpio::Pin)
                    pub type $Alias = $crate::gpio::Pin<
                        $crate::gpio::bank0::$Id,
                        $crate::gpio::$Mode
                    >;

                    $( #[$attr] )*
                    #[doc = "[DynPinId](rp2040_hal::gpio::DynPinId) "]
                    #[doc = "for the `" $Alias "` alias."]
                    pub const [<$Alias:snake:upper _ID>]: $crate::gpio::DynPinId =
                    <$crate::gpio::bank0::$Id as $crate::gpio::PinId>::DYN;

                    $( #[$attr] )*
                    #[doc = "[DynPinMode]rp2040_hal::gpio::DynPinMode) "]
                    #[doc = "for the `" $Alias "` alias."]
                    pub const [<$Alias:snake:upper _MODE>]: $crate::gpio::DynPinMode =
                    <$crate::gpio::$Mode as $crate::gpio::PinMode>::DYN;
                )+
            )?
        }
    };
}
