//! Pulse Width Modulation (PWM)
//!
//! First you must create a Slices struct which contains all the pwm slices.
//!
//! ```no_run
//! use rp2040_hal::{prelude::*, pwm::{InputHighRunning, Slices}};
//!
//!
//! let mut pac = rp2040_pac::Peripherals::take().unwrap();
//!
//! // Init PWMs
//! let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
//!
//! // Configure PWM4
//! let mut pwm = pwm_slices.pwm4;
//! pwm.set_ph_correct();
//! pwm.enable();
//!
//! // Set to run when b channel is high
//! let pwm  = pwm.into_mode::<InputHighRunning>();
//! ```
//!
//! Once you have the PWM slice struct, you can add individual pins:
//!
//! ```no_run
//! # use rp2040_hal::{prelude::*, gpio::Pins, Sio, pwm::{InputHighRunning, Slices}};
//! # let mut pac = rp2040_pac::Peripherals::take().unwrap();
//! # let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
//! # let mut pwm = pwm_slices.pwm4.into_mode::<InputHighRunning>();
//! # let mut pac = rp2040_pac::Peripherals::take().unwrap();
//! #
//! # let sio = Sio::new(pac.SIO);
//! # let pins = Pins::new(
//! #     pac.IO_BANK0,
//! #     pac.PADS_BANK0,
//! #     sio.gpio_bank0,
//! #     &mut pac.RESETS,
//! # );
//! #
//! use embedded_hal::pwm::SetDutyCycle;
//!
//! // Use B channel (which inputs from GPIO 25)
//! let mut channel_b = pwm.channel_b;
//! let channel_pin_b = channel_b.input_from(pins.gpio25);
//!
//! // Use A channel (which outputs to GPIO 24)
//! let mut channel_a = pwm.channel_a;
//! let channel_pin_a = channel_a.output_to(pins.gpio24);
//!
//! // Set duty cycle
//! channel_a.set_duty_cycle(0x00ff);
//! let max_duty_cycle = channel_a.max_duty_cycle();
//! channel_a.set_inverted(); // Invert the output
//! channel_a.clr_inverted(); // Don't invert the output
//! ```
//!
//! The following configuration options are also available:
//!
//! ```no_run
//! # use rp2040_hal::{prelude::*, pwm::Slices};
//! # let mut pac = rp2040_pac::Peripherals::take().unwrap();
//! # let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
//! # let mut pwm = pwm_slices.pwm4;
//! pwm.set_ph_correct(); // Run in phase correct mode
//! pwm.clr_ph_correct(); // Don't run in phase correct mode
//!
//! pwm.set_div_int(1u8); // To set integer part of clock divider
//! pwm.set_div_frac(0u8); // To set fractional part of clock divider
//!
//! pwm.get_top(); // To get the TOP register
//! pwm.set_top(u16::MAX); // To set the TOP register
//!
//! ```
//!
//! default_config() sets ph_correct to false, the clock divider to 1, does not invert the output, sets top to 65535, and resets the counter.
//! min_config() leaves those registers in the state they were before it was called (Careful, this can lead to unexpected behavior)
//! It's recommended to only call min_config() after calling default_config() on a pin that shares a PWM block.

use core::convert::Infallible;
use core::marker::PhantomData;

use embedded_dma::Word;
use embedded_hal::pwm::{ErrorType, SetDutyCycle};

use crate::{
    atomic_register_access::{write_bitmask_clear, write_bitmask_set},
    dma::{EndlessWriteTarget, WriteTarget},
    gpio::{bank0::*, AnyPin, FunctionPwm, Pin, ValidFunction},
    pac::{self, dma::ch::ch_al1_ctrl::TREQ_SEL_A, PWM},
    resets::SubsystemReset,
    typelevel::{Is, Sealed},
};

pub mod dyn_slice;
pub use dyn_slice::*;

mod reg;

use reg::RegisterInterface;

/// Used to pin traits to a specific channel (A or B)
pub trait ChannelId: Sealed {
    /// Corresponding [`DynChannelId`]
    const DYN: DynChannelId;
}

/// Channel A
///
/// These are attached to the even gpio pins and can only do PWM output
pub enum A {}

/// Channel B
///
/// These are attached to the odd gpio pins and can do PWM output and edge counting for input
pub enum B {}

impl ChannelId for A {
    const DYN: DynChannelId = DynChannelId::A;
}
impl ChannelId for B {
    const DYN: DynChannelId = DynChannelId::B;
}
impl Sealed for A {}
impl Sealed for B {}

/// Counter is free-running, and will count continuously whenever the slice is enabled
pub struct FreeRunning;
/// Count continuously when a high level is detected on the B pin
pub struct InputHighRunning;
/// Count once with each rising edge detected on the B pin
pub struct CountRisingEdge;
/// Count once with each falling edge detected on the B pin
pub struct CountFallingEdge;

/// Type-level marker for tracking which slice modes are valid for which slices
pub trait ValidSliceMode<I: SliceId>: Sealed + SliceMode {}

/// Type-level marker for tracking which slice modes are valid for which slices
pub trait ValidSliceInputMode<I: SliceId>: Sealed + ValidSliceMode<I> {}

/// Mode for slice
pub trait SliceMode: Sealed + Sized {
    /// Corresponding [`DynSliceMode`]
    const DYN: DynSliceMode;
}

impl Sealed for FreeRunning {}
impl SliceMode for FreeRunning {
    const DYN: DynSliceMode = DynSliceMode::FreeRunning;
}
impl Sealed for InputHighRunning {}
impl SliceMode for InputHighRunning {
    const DYN: DynSliceMode = DynSliceMode::InputHighRunning;
}
impl Sealed for CountRisingEdge {}
impl SliceMode for CountRisingEdge {
    const DYN: DynSliceMode = DynSliceMode::CountRisingEdge;
}
impl Sealed for CountFallingEdge {}
impl SliceMode for CountFallingEdge {
    const DYN: DynSliceMode = DynSliceMode::CountFallingEdge;
}

impl<I: SliceId> ValidSliceMode<I> for FreeRunning {}
impl<I: SliceId> ValidSliceMode<I> for InputHighRunning {}
impl<I: SliceId> ValidSliceMode<I> for CountRisingEdge {}
impl<I: SliceId> ValidSliceMode<I> for CountFallingEdge {}
impl<I: SliceId> ValidSliceInputMode<I> for InputHighRunning {}
impl<I: SliceId> ValidSliceInputMode<I> for CountRisingEdge {}
impl<I: SliceId> ValidSliceInputMode<I> for CountFallingEdge {}

//==============================================================================
//  Slice IDs
//==============================================================================

/// Type-level `enum` for slice IDs
pub trait SliceId: Sealed {
    /// Corresponding [`DynSliceId`]
    const DYN: DynSliceId;
    /// [`SliceMode`] at reset
    type Reset;

    /// Get DREQ number of PWM wrap.
    const WRAP_DREQ: u8 = TREQ_SEL_A::PWM_WRAP0 as u8 + Self::DYN.num;
}

macro_rules! slice_id {
    ($Id:ident, $NUM:literal, $reset : ident) => {
        $crate::paste::paste! {
            #[doc = "Slice ID representing slice " $NUM]
            pub enum $Id {}
            impl Sealed for $Id {}
            impl SliceId for $Id {
                type Reset = $reset;
                const DYN: DynSliceId = DynSliceId { num: $NUM };
            }
        }
    };
}

//==============================================================================
//  AnySlice
//==============================================================================

/// Type class for [`Slice`] types
///
/// This trait uses the [`AnyKind`] trait pattern to create a [type class] for
/// [`Slice`] types. See the `AnyKind` documentation for more details on the
/// pattern.
///
/// [`AnyKind`]: crate::typelevel#anykind-trait-pattern
/// [type class]: crate::typelevel#type-classes
pub trait AnySlice
where
    Self: Sealed,
    Self: Is<Type = SpecificSlice<Self>>,
    <Self as AnySlice>::Mode: ValidSliceMode<<Self as AnySlice>::Id>,
{
    /// [`SliceId`] of the corresponding [`Slice`]
    type Id: SliceId;
    /// [`SliceMode`] of the corresponding [`Slice`]
    type Mode: SliceMode;
}

impl<S, M> Sealed for Slice<S, M>
where
    S: SliceId,
    M: ValidSliceMode<S>,
{
}

impl<S, M> AnySlice for Slice<S, M>
where
    S: SliceId,
    M: ValidSliceMode<S>,
{
    type Id = S;
    type Mode = M;
}

/// Type alias to recover the specific [`Slice`] type from an implementation of
/// [`AnySlice`]
///
/// See the [`AnyKind`] documentation for more details on the pattern.
///
/// [`AnyKind`]: crate::typelevel#anykind-trait-pattern
type SpecificSlice<S> = Slice<<S as AnySlice>::Id, <S as AnySlice>::Mode>;

//==============================================================================
//  Registers
//==============================================================================

/// Provide a safe register interface for [`Slice`]s
///
/// This `struct` takes ownership of a [`SliceId`] and provides an API to
/// access the corresponding registers.
struct Registers<I: SliceId> {
    id: PhantomData<I>,
}

// [`Registers`] takes ownership of the [`SliceId`], and [`Slice`] guarantees that
// each slice is a singleton, so this implementation is safe.
unsafe impl<I: SliceId> RegisterInterface for Registers<I> {
    #[inline]
    fn id(&self) -> DynSliceId {
        I::DYN
    }
}

impl<I: SliceId> Registers<I> {
    /// Create a new instance of [`Registers`]
    ///
    /// # Safety
    ///
    /// Users must never create two simultaneous instances of this `struct` with
    /// the same [`SliceId`]
    #[inline]
    unsafe fn new() -> Self {
        Registers { id: PhantomData }
    }

    /// Provide a type-level equivalent for the
    /// [`RegisterInterface::change_mode`] method.
    #[inline]
    fn change_mode<M: ValidSliceMode<I>>(&mut self) {
        RegisterInterface::do_change_mode(self, M::DYN);
    }
}

/// Pwm slice
pub struct Slice<I, M>
where
    I: SliceId,
    M: ValidSliceMode<I>,
{
    regs: Registers<I>,
    mode: PhantomData<M>,
    /// Channel A (always output)
    pub channel_a: Channel<Self, A>,
    /// Channel B (input or output)
    pub channel_b: Channel<Self, B>,
}

impl<I, M> Slice<I, M>
where
    I: SliceId,
    M: ValidSliceMode<I>,
{
    /// Create a new [`Slice`]
    ///
    /// # Safety
    ///
    /// Each [`Slice`] must be a singleton. For a given [`SliceId`], there must be
    /// at most one corresponding [`Slice`] in existence at any given time.
    /// Violating this requirement is `unsafe`.
    #[inline]
    pub(crate) unsafe fn new() -> Slice<I, M> {
        Slice {
            regs: Registers::new(),
            mode: PhantomData,
            channel_a: Channel::new(0),
            channel_b: Channel::new(0),
        }
    }

    /// Convert the slice to the requested [`SliceMode`]
    #[inline]
    pub fn into_mode<N: ValidSliceMode<I>>(mut self) -> Slice<I, N> {
        if N::DYN != M::DYN {
            self.regs.change_mode::<N>();
        }
        // Safe because we drop the existing slice
        unsafe { Slice::new() }
    }

    /// Set a default config for the slice
    pub fn default_config(&mut self) {
        self.regs.write_ph_correct(false);
        self.regs.write_div_int(1); // No divisor
        self.regs.write_div_frac(0); // No divisor
        self.regs.write_inv_a(false); //Don't invert the channel
        self.regs.write_inv_b(false); //Don't invert the channel
        self.regs.write_top(0xfffe); // Wrap at 0xfffe, so cc = 0xffff can indicate 100% duty cycle
        self.regs.write_ctr(0x0000); //Reset the counter
        self.regs.write_cc_a(0); //Default duty cycle of 0%
        self.regs.write_cc_b(0); //Default duty cycle of 0%
    }

    /// Advance the phase with one count
    ///
    /// Counter must be running at less than full speed (div_int + div_frac / 16 > 1)
    #[inline]
    pub fn advance_phase(&mut self) {
        self.regs.advance_phase()
    }

    /// Retard the phase with one count
    ///
    /// Counter must be running at less than full speed (div_int + div_frac / 16 > 1)
    #[inline]
    pub fn retard_phase(&mut self) {
        self.regs.retard_phase()
    }

    /// Enable phase correct mode
    #[inline]
    pub fn set_ph_correct(&mut self) {
        self.regs.write_ph_correct(true)
    }

    /// Disables phase correct mode
    #[inline]
    pub fn clr_ph_correct(&mut self) {
        self.regs.write_ph_correct(false)
    }

    /// Enable slice
    #[inline]
    pub fn enable(&mut self) {
        self.regs.write_enable(true);
    }

    /// Disable slice
    #[inline]
    pub fn disable(&mut self) {
        self.regs.write_enable(false)
    }

    /// Sets the integer part of the clock divider
    #[inline]
    pub fn set_div_int(&mut self, value: u8) {
        self.regs.write_div_int(value)
    }

    /// Sets the fractional part of the clock divider
    #[inline]
    pub fn set_div_frac(&mut self, value: u8) {
        self.regs.write_div_frac(value)
    }

    /// Get the counter register value
    #[inline]
    pub fn get_counter(&self) -> u16 {
        self.regs.read_ctr()
    }

    /// Set the counter register value
    #[inline]
    pub fn set_counter(&mut self, value: u16) {
        self.regs.write_ctr(value)
    }

    /// Get the top register value
    #[inline]
    pub fn get_top(&self) -> u16 {
        self.regs.read_top()
    }

    /// Sets the top register value
    ///
    /// Don't set this to 0xffff if you need true 100% duty cycle:
    ///
    /// The CC register, which is used to configure the duty cycle,
    /// must be set to TOP + 1 for 100% duty cycle, but also is a
    /// 16 bit register.
    ///
    /// In case you do set TOP to 0xffff, [`SetDutyCycle::set_duty_cycle`]
    /// will slightly violate the trait's documentation, as
    /// `SetDutyCycle::set_duty_cycle_fully_on` and other calls that
    /// should lead to 100% duty cycle will only reach a duty cycle of
    /// about 99.998%.
    #[inline]
    pub fn set_top(&mut self, value: u16) {
        self.regs.write_top(value)
    }

    /// Create the interrupt bitmask corresponding to this slice
    #[inline]
    fn bitmask(&self) -> u32 {
        1 << I::DYN.num
    }

    /// Enable the PWM_IRQ_WRAP interrupt when this slice overflows.
    #[inline]
    pub fn enable_interrupt(&mut self) {
        unsafe {
            let pwm = &(*pac::PWM::ptr());
            let reg = pwm.inte().as_ptr();
            write_bitmask_set(reg, self.bitmask());
        }
    }

    /// Disable the PWM_IRQ_WRAP interrupt for this slice.
    #[inline]
    pub fn disable_interrupt(&mut self) {
        unsafe {
            let pwm = &(*pac::PWM::ptr());
            let reg = pwm.inte().as_ptr();
            write_bitmask_clear(reg, self.bitmask());
        };
    }

    /// Did this slice trigger an overflow interrupt?
    ///
    /// This reports the raw interrupt flag, without considering masking or
    /// forcing bits. It may return true even if the interrupt is disabled
    /// or false even if the interrupt is forced.
    #[inline]
    pub fn has_overflown(&self) -> bool {
        let mask = self.bitmask();
        unsafe { (*pac::PWM::ptr()).intr().read().bits() & mask == mask }
    }

    /// Mark the interrupt handled for this slice.
    #[inline]
    pub fn clear_interrupt(&mut self) {
        unsafe { (*pac::PWM::ptr()).intr().write(|w| w.bits(self.bitmask())) };
    }

    /// Force the interrupt. This bit is not cleared by hardware and must be manually cleared to
    /// stop the interrupt from continuing to be asserted.
    #[inline]
    pub fn force_interrupt(&mut self) {
        unsafe {
            let pwm = &(*pac::PWM::ptr());
            let reg = pwm.intf().as_ptr();
            write_bitmask_set(reg, self.bitmask());
        }
    }

    /// Clear force interrupt. This bit is not cleared by hardware and must be manually cleared to
    /// stop the interrupt from continuing to be asserted.
    #[inline]
    pub fn clear_force_interrupt(&mut self) {
        unsafe {
            let pwm = &(*pac::PWM::ptr());
            let reg = pwm.intf().as_ptr();
            write_bitmask_clear(reg, self.bitmask());
        }
    }
}

macro_rules! pwm {
    ($PWMX:ident, [
        $($SXi:ident: ($slice:literal, [$($pin_a:ident, $pin_b:ident),*], $i:expr)),+
    ]) => {
        $(
            slice_id!($SXi, $slice, FreeRunning);

            $(
                impl ValidPwmOutputPin<$SXi, A> for $pin_a {}
                impl ValidPwmOutputPin<$SXi, B> for $pin_b {}
                impl ValidPwmInputPin<$SXi> for $pin_b {}
            )*
        )+

        $crate::paste::paste!{

            /// Collection of all the individual [`Slices`]s
            pub struct Slices {
                _pwm: $PWMX,
                $(
                    #[doc = "Slice " $SXi]
                    pub [<$SXi:lower>] : Slice<$SXi,<$SXi as SliceId>::Reset>,
                )+
            }

            impl Slices {
                /// Take ownership of the PAC peripheral and split it into discrete [`Slice`]s
                pub fn new(pwm: $PWMX, reset : &mut crate::pac::RESETS) -> Self {
                    pwm.reset_bring_up(reset);
                    unsafe {
                        Self {
                            _pwm: pwm,
                            $(
                                [<$SXi:lower>]: Slice::new(),
                            )+
                        }
                    }
                }
            }
        }
    }
}

pwm! {
    PWM, [
        Pwm0: (0, [Gpio0, Gpio1, Gpio16, Gpio17], 0),
        Pwm1: (1, [Gpio2, Gpio3, Gpio18, Gpio19], 1),
        Pwm2: (2, [Gpio4, Gpio5, Gpio20, Gpio21], 2),
        Pwm3: (3, [Gpio6, Gpio7, Gpio22, Gpio23], 3),
        Pwm4: (4, [Gpio8, Gpio9, Gpio24, Gpio25], 4),
        Pwm5: (5, [Gpio10, Gpio11, Gpio26, Gpio27], 5),
        Pwm6: (6, [Gpio12, Gpio13, Gpio28, Gpio29], 6),
        Pwm7: (7, [Gpio14, Gpio15], 7)
    ]
}

/// Marker trait for valid input pins (Channel B only)
pub trait ValidPwmInputPin<S: SliceId>: ValidFunction<FunctionPwm> + Sealed {}
/// Marker trait for valid output pins
pub trait ValidPwmOutputPin<S: SliceId, C: ChannelId>: ValidFunction<FunctionPwm> + Sealed {}

impl Slices {
    /// Free the pwm registers from the pwm hal struct while consuming it.
    pub fn free(self) -> PWM {
        self._pwm
    }

    /// Enable multiple slices at the same time to make their counters sync up.
    ///
    /// You still need to call `slice` to get an actual slice
    pub fn enable_simultaneous(&mut self, bits: u8) {
        // Enable multiple slices at the same time
        unsafe {
            let reg = self._pwm.en().as_ptr();
            write_bitmask_set(reg, bits as u32);
        }
    }

    // /// Get pwm slice based on gpio pin
    // pub fn borrow_mut_from_pin<
    //     S: SliceId,
    //     C: ChannelId,
    //     G: PinId + BankPinId + ValidPwmOutputPin<S, C>,
    //     PM: PinMode + ValidPinMode<G>,
    //     SM:  ValidSliceMode<S>,
    // >(&mut self, _: &Pin<G, PM>) -> &mut Slice<S, SM>{
    //     match S::DYN {
    //         DynSliceId{num} if num == 0 => &mut self.pwm0,
    //         DynSliceId{num} if num == 1 => &mut self.pwm1,
    //         DynSliceId{num} if num == 2 => &mut self.pwm2,
    //         DynSliceId{num} if num == 3 => &mut self.pwm3,
    //         DynSliceId{num} if num == 4 => &mut self.pwm4,
    //         DynSliceId{num} if num == 5 => &mut self.pwm5,
    //         DynSliceId{num} if num == 6 => &mut self.pwm6,
    //         DynSliceId{num} if num == 7 => &mut self.pwm7,
    //         _ => unreachable!()
    //     }
    // }
}

/// A Channel from the Pwm subsystem.
///
/// Its attached to one of the eight slices and can be an A or B side channel
pub struct Channel<S: AnySlice, C: ChannelId> {
    regs: Registers<S::Id>,
    slice_mode: PhantomData<S::Mode>,
    channel_id: PhantomData<C>,
    duty_cycle: u16,
    enabled: bool,
}

impl<S: AnySlice, C: ChannelId> Channel<S, C> {
    pub(super) unsafe fn new(duty_cycle: u16) -> Self {
        Channel {
            regs: Registers::new(),
            slice_mode: PhantomData,
            channel_id: PhantomData,
            duty_cycle, // stores the duty cycle while the channel is disabled
            enabled: true,
        }
    }
}

impl<S: AnySlice, C: ChannelId> Sealed for Channel<S, C> {}

impl<S: AnySlice> embedded_hal_0_2::PwmPin for Channel<S, A> {
    type Duty = u16;

    fn disable(&mut self) {
        self.set_enabled(false);
    }

    fn enable(&mut self) {
        self.set_enabled(true);
    }

    fn get_duty(&self) -> Self::Duty {
        if self.enabled {
            self.regs.read_cc_a()
        } else {
            self.duty_cycle
        }
    }

    fn get_max_duty(&self) -> Self::Duty {
        SetDutyCycle::max_duty_cycle(self)
    }

    fn set_duty(&mut self, duty: Self::Duty) {
        let _ = SetDutyCycle::set_duty_cycle(self, duty);
    }
}

impl<S: AnySlice> embedded_hal_0_2::PwmPin for Channel<S, B> {
    type Duty = u16;

    fn disable(&mut self) {
        self.set_enabled(false);
    }

    fn enable(&mut self) {
        self.set_enabled(true);
    }

    fn get_duty(&self) -> Self::Duty {
        if self.enabled {
            self.regs.read_cc_b()
        } else {
            self.duty_cycle
        }
    }

    fn get_max_duty(&self) -> Self::Duty {
        SetDutyCycle::max_duty_cycle(self)
    }

    fn set_duty(&mut self, duty: Self::Duty) {
        let _ = SetDutyCycle::set_duty_cycle(self, duty);
    }
}

impl<S: AnySlice> ErrorType for Channel<S, A> {
    type Error = Infallible;
}

impl<S: AnySlice> SetDutyCycle for Channel<S, A> {
    fn max_duty_cycle(&self) -> u16 {
        self.regs.read_top().saturating_add(1)
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.duty_cycle = duty;
        if self.enabled {
            self.regs.write_cc_a(duty)
        }
        Ok(())
    }
}

impl<S: AnySlice> ErrorType for Channel<S, B> {
    type Error = Infallible;
}

impl<S: AnySlice> SetDutyCycle for Channel<S, B> {
    fn max_duty_cycle(&self) -> u16 {
        self.regs.read_top().saturating_add(1)
    }

    fn set_duty_cycle(&mut self, duty: u16) -> Result<(), Self::Error> {
        self.duty_cycle = duty;
        if self.enabled {
            self.regs.write_cc_b(duty)
        }
        Ok(())
    }
}

impl<S: AnySlice> Channel<S, A> {
    /// Enable or disable the PWM channel
    pub fn set_enabled(&mut self, enable: bool) {
        if enable && !self.enabled {
            // Restore the duty cycle.
            self.regs.write_cc_a(self.duty_cycle);
            self.enabled = true;
        } else if !enable && self.enabled {
            // We can't disable it without disturbing the other channel so this
            // just sets the duty cycle to zero.
            self.duty_cycle = self.regs.read_cc_a();
            self.regs.write_cc_a(0);
            self.enabled = false;
        }
    }

    /// Capture a gpio pin and use it as pwm output for channel A
    pub fn output_to<P: AnyPin>(&mut self, pin: P) -> Pin<P::Id, FunctionPwm, P::Pull>
    where
        P::Id: ValidPwmOutputPin<S::Id, A>,
    {
        pin.into().into_function()
    }

    /// Invert channel output
    #[inline]
    pub fn set_inverted(&mut self) {
        self.regs.write_inv_a(true)
    }

    /// Stop inverting channel output
    #[inline]
    pub fn clr_inverted(&mut self) {
        self.regs.write_inv_a(false)
    }
}

impl<S: AnySlice> Channel<S, B> {
    /// Enable or disable the PWM channel
    pub fn set_enabled(&mut self, enable: bool) {
        if enable && !self.enabled {
            // Restore the duty cycle.
            self.regs.write_cc_b(self.duty_cycle);
            self.enabled = true;
        } else if !enable && self.enabled {
            // We can't disable it without disturbing the other channel so this
            // just sets the duty cycle to zero.
            self.duty_cycle = self.regs.read_cc_b();
            self.regs.write_cc_b(0);
            self.enabled = false;
        }
    }

    /// Capture a gpio pin and use it as pwm output for channel B
    pub fn output_to<P: AnyPin>(&mut self, pin: P) -> Pin<P::Id, FunctionPwm, P::Pull>
    where
        P::Id: ValidPwmOutputPin<S::Id, B>,
    {
        pin.into().into_function()
    }

    /// Invert channel output
    #[inline]
    pub fn set_inverted(&mut self) {
        self.regs.write_inv_b(true)
    }

    /// Stop inverting channel output
    #[inline]
    pub fn clr_inverted(&mut self) {
        self.regs.write_inv_b(false)
    }
}

impl<S: AnySlice> Channel<S, B>
where
    S::Mode: ValidSliceInputMode<S::Id>,
{
    /// Capture a gpio pin and use it as pwm input for channel B
    pub fn input_from<P: AnyPin>(&mut self, pin: P) -> Pin<P::Id, FunctionPwm, P::Pull>
    where
        P::Id: ValidPwmInputPin<S::Id>,
    {
        pin.into().into_function()
    }
}

impl<S: SliceId, M: ValidSliceMode<S>> Slice<S, M> {
    /// Capture a gpio pin and use it as pwm output
    pub fn output_to<P: AnyPin, C: ChannelId>(&mut self, pin: P) -> Pin<P::Id, FunctionPwm, P::Pull>
    where
        P::Id: ValidPwmOutputPin<S, C>,
    {
        pin.into().into_function()
    }
}

impl<S: SliceId, M: ValidSliceInputMode<S>> Slice<S, M> {
    /// Capture a gpio pin and use it as pwm input for channel B
    pub fn input_from<P: AnyPin>(&mut self, pin: P) -> Pin<P::Id, FunctionPwm, P::Pull>
    where
        P::Id: ValidPwmInputPin<S>,
    {
        pin.into().into_function()
    }
}

/// Type representing DMA access to PWM cc register.
///
/// Both channels are accessed together, because of narrow write replication.
///
/// ```no_run
/// use cortex_m::singleton;
/// use rp2040_hal::dma::{double_buffer, DMAExt};
/// use rp2040_hal::pwm::{CcFormat, SliceDmaWrite, Slices};
///
///
/// let mut pac = rp2040_pac::Peripherals::take().unwrap();
///
/// // Init PWMs
/// let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
///
/// // Configure PWM4
/// let mut pwm = pwm_slices.pwm4;
/// pwm.enable();
///
/// let buf = singleton!(: [CcFormat; 4] = [CcFormat{a: 0x1000, b: 0x9000}; 4]).unwrap();
/// let buf2 = singleton!(: [CcFormat; 4] = [CcFormat{a: 0xf000, b: 0x5000}; 4]).unwrap();
///
/// let dma = pac.DMA.split(&mut pac.RESETS);
///
/// let dma_pwm = SliceDmaWrite::from(pwm);
///
/// let dma_conf = double_buffer::Config::new((dma.ch0, dma.ch1), buf, dma_pwm.cc);
/// ```
pub struct SliceDmaWriteCc<S: SliceId, M: ValidSliceMode<S>> {
    slice: PhantomData<S>,
    mode: PhantomData<M>,
}

/// Type representing DMA access to PWM top register.
///
/// ```no_run
/// use cortex_m::{prelude::*, singleton};
/// use rp2040_hal::dma::{double_buffer, DMAExt};
/// use rp2040_hal::pwm::{SliceDmaWrite, Slices, TopFormat};
///
///
/// let mut pac = rp2040_pac::Peripherals::take().unwrap();
///
/// // Init PWMs
/// let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
///
/// // Configure PWM4
/// let mut pwm = pwm_slices.pwm4;
/// pwm.enable();
///
/// // Just set to something mesurable.
/// pwm.channel_a.set_duty(0x1000);
/// pwm.channel_b.set_duty(0x1000);
///
/// let buf = singleton!(: [TopFormat; 4] = [TopFormat::new(0x7fff); 4]).unwrap();
/// let buf2 = singleton!(: [TopFormat; 4] = [TopFormat::new(0xfffe); 4]).unwrap();
///
/// let dma = pac.DMA.split(&mut pac.RESETS);
///
/// // Reserve PWM slice for dma.
/// let dma_pwm = SliceDmaWrite::from(pwm);
///
/// let dma_conf = double_buffer::Config::new((dma.ch0, dma.ch1), buf, dma_pwm.top);
/// ```
pub struct SliceDmaWriteTop<S: SliceId, M: ValidSliceMode<S>> {
    slice: PhantomData<S>,
    mode: PhantomData<M>,
}

/// PWM slice while used for DMA writes.
/// ```no_run
/// use rp2040_hal::{prelude::*, pwm::{SliceDmaWrite, Slices}};
///
///
/// let mut pac = rp2040_pac::Peripherals::take().unwrap();
///
/// // Init PWMs
/// let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
///
/// // Configure PWM4
/// let mut pwm = pwm_slices.pwm4;
/// pwm.enable();
///
/// // Use for DMA usage
/// let dma_pwm = SliceDmaWrite::from(pwm);
/// ```
///
pub struct SliceDmaWrite<S: SliceId, M: ValidSliceMode<S>> {
    /// Part for top writes.
    pub top: SliceDmaWriteTop<S, M>,

    /// Part for cc writes.
    pub cc: SliceDmaWriteCc<S, M>,
    slice: Slice<S, M>,
}

impl<S: SliceId, M: ValidSliceMode<S>> From<Slice<S, M>> for SliceDmaWrite<S, M> {
    fn from(value: Slice<S, M>) -> Self {
        Self {
            slice: value,
            top: SliceDmaWriteTop {
                slice: PhantomData,
                mode: PhantomData,
            },
            cc: SliceDmaWriteCc {
                slice: PhantomData,
                mode: PhantomData,
            },
        }
    }
}

impl<S: SliceId, M: ValidSliceMode<S>> From<SliceDmaWrite<S, M>> for Slice<S, M> {
    fn from(value: SliceDmaWrite<S, M>) -> Self {
        value.slice
    }
}

/// Format for DMA transfers to PWM CC register.
#[derive(Clone, Copy, Eq, PartialEq)]
#[repr(C)]
#[repr(align(4))]
pub struct CcFormat {
    /// CC register part for channel a.
    pub a: u16,
    /// CC register part for channel b.
    pub b: u16,
}

unsafe impl Word for CcFormat {}

/// Format for DMA transfers to PWM TOP register.
///
/// It is forbidden to use it as DMA write destination,
/// it is safe but it might not be compatible with a future use of reserved register fields.
#[derive(Clone, Copy, Eq)]
#[repr(C)]
#[repr(align(4))]
pub struct TopFormat {
    /// Valid register part.
    pub top: u16,
    /// Reserved part.
    /// Should always be zero
    reserved: u16,
}

impl PartialEq<TopFormat> for TopFormat {
    fn eq(&self, other: &TopFormat) -> bool {
        self.top == other.top
    }
}

impl TopFormat {
    /// Create a valid value.
    pub fn new(top: u16) -> Self {
        TopFormat { top, reserved: 0 }
    }
}

impl Default for TopFormat {
    fn default() -> Self {
        Self::new(u16::MAX)
    }
}

unsafe impl Word for TopFormat {}

/// Safety: tx_address_count points to a register which is always a valid
/// write target.
unsafe impl<S: SliceId, M: ValidSliceMode<S>> WriteTarget for SliceDmaWriteCc<S, M> {
    type TransmittedWord = CcFormat;

    fn tx_treq() -> Option<u8> {
        Some(S::WRAP_DREQ)
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        let regs = Registers {
            id: PhantomData::<S> {},
        };
        (regs.ch().cc().as_ptr() as u32, u32::MAX)
    }

    fn tx_increment(&self) -> bool {
        false
    }
}

/// Safety: tx_address_count points to a register which is always a valid
/// write target.
unsafe impl<S: SliceId, M: ValidSliceMode<S>> WriteTarget for SliceDmaWriteTop<S, M> {
    type TransmittedWord = TopFormat;

    fn tx_treq() -> Option<u8> {
        Some(S::WRAP_DREQ)
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        let regs = Registers {
            id: PhantomData::<S> {},
        };
        (regs.ch().top().as_ptr() as u32, u32::MAX)
    }

    fn tx_increment(&self) -> bool {
        false
    }
}

impl<S: SliceId, M: ValidSliceMode<S>> EndlessWriteTarget for SliceDmaWriteCc<S, M> {}
impl<S: SliceId, M: ValidSliceMode<S>> EndlessWriteTarget for SliceDmaWriteTop<S, M> {}
