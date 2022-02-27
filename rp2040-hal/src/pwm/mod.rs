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
//! use embedded_hal::PwmPin;
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
//! channel_a.set_duty(0x00ff);
//! channel_a.get_duty();
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

use core::marker::PhantomData;

use crate::{
    gpio::{
        bank0::*, FunctionClock, FunctionI2C, FunctionPio0, FunctionPio1, FunctionPwm, FunctionSpi,
        FunctionUart, FunctionUsbAux, FunctionXip, Input, InputConfig, Output, OutputConfig, Pin,
        PinId, PinMode, ValidPinMode,
    },
    resets::SubsystemReset,
    typelevel::Sealed,
};
use embedded_hal::PwmPin;
use pac::PWM;

use crate::atomic_register_access::{write_bitmask_clear, write_bitmask_set};

pub mod dyn_slice;
pub use dyn_slice::*;

mod reg;

use reg::RegisterInterface;

/// Used to pin traits to a specific channel (A or B)
pub trait ChannelId: Sealed {
    /// Corresponding [`DynChannelId`](dyn_slice::DynChannelId)
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
pub trait ValidSliceMode<I: SliceId>: Sealed {}

/// Type-level marker for tracking which slice modes are valid for which slices
pub trait ValidSliceInputMode<I: SliceId>: Sealed + ValidSliceMode<I> {}

/// Mode for slice
pub trait SliceMode: Sealed + Sized {
    /// Corresponding [`DynSliceMode`](dyn_slice::DynSliceMode)
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
    /// Corresponding [`DynSliceId`](dyn_slice::DynSliceId)
    const DYN: DynSliceId;
    /// [`SliceMode`] at reset
    type Reset;
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
    fn change_mode<M: SliceMode + ValidSliceMode<I>>(&mut self) {
        RegisterInterface::do_change_mode(self, M::DYN);
    }
}

/// Pwm slice
pub struct Slice<I, M>
where
    I: SliceId,
    M: SliceMode + ValidSliceMode<I>,
{
    regs: Registers<I>,
    mode: PhantomData<M>,
    /// Channel A (always output)
    pub channel_a: Channel<I, M, A>,
    /// Channel B (input or output)
    pub channel_b: Channel<I, M, B>,
}

impl<I, M> Slice<I, M>
where
    I: SliceId,
    M: SliceMode + ValidSliceMode<I>,
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
    pub fn into_mode<N: SliceMode + ValidSliceMode<I>>(mut self) -> Slice<I, N> {
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
        self.regs.write_top(0xffff); // Wrap at max
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
            let reg = (&pwm.inte).as_ptr();
            write_bitmask_set(reg, self.bitmask());
        }
    }

    /// Disable the PWM_IRQ_WRAP interrupt for this slice.
    #[inline]
    pub fn disable_interrupt(&mut self) {
        unsafe {
            let pwm = &(*pac::PWM::ptr());
            let reg = (&pwm.inte).as_ptr();
            write_bitmask_clear(reg, self.bitmask());
        };
    }

    /// Did this slice trigger an overflow interrupt?
    #[inline]
    pub fn has_overflown(&self) -> bool {
        let mask = self.bitmask();
        unsafe { (*pac::PWM::ptr()).ints.read().bits() & mask == mask }
    }

    /// Mark the interrupt handled for this slice.
    #[inline]
    pub fn clear_interrupt(&mut self) {
        unsafe { (*pac::PWM::ptr()).intr.write(|w| w.bits(self.bitmask())) };
    }

    /// Force the interrupt. This bit is not cleared by hardware and must be manually cleared to
    /// stop the interrupt from continuing to be asserted.
    #[inline]
    pub fn force_interrupt(&mut self) {
        unsafe {
            let pwm = &(*pac::PWM::ptr());
            let reg = (&pwm.intf).as_ptr();
            write_bitmask_set(reg, self.bitmask());
        }
    }

    /// Clear force interrupt. This bit is not cleared by hardware and must be manually cleared to
    /// stop the interrupt from continuing to be asserted.
    #[inline]
    pub fn clear_force_interrupt(&mut self) {
        unsafe {
            let pwm = &(*pac::PWM::ptr());
            let reg = (&pwm.intf).as_ptr();
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
                pub fn new(pwm: $PWMX, reset : &mut pac::RESETS) -> Self {
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

/// Marker trait for valid output pins
pub trait ValidPwmInputPin<S: SliceId>: Sealed {}
/// Marker trait for valid input pins (Channel B only)
pub trait ValidPwmOutputPin<S: SliceId, C: ChannelId>: Sealed {}

/// Make sure we can't free an GPIO pin while still keeping it attached to pwm
/// TODO: Maybe FunctionPWM should be private?
pub trait NonPwmPinMode: Sealed {}

impl NonPwmPinMode for FunctionClock {}
impl NonPwmPinMode for FunctionI2C {}
impl NonPwmPinMode for FunctionPio0 {}
impl NonPwmPinMode for FunctionPio1 {}
impl NonPwmPinMode for FunctionSpi {}
impl NonPwmPinMode for FunctionUart {}
impl NonPwmPinMode for FunctionUsbAux {}
impl NonPwmPinMode for FunctionXip {}
impl<C: InputConfig> NonPwmPinMode for Input<C> {}
impl<C: OutputConfig> NonPwmPinMode for Output<C> {}

/// Stores the attached gpio pin.
///
/// This value can be ignored/dropped or stored to retrieve the original pin struct
pub struct PwmPinToken<G: PinId + BankPinId> {
    pin: Pin<G, FunctionPwm>,
}

impl<G: PinId + BankPinId> PwmPinToken<G> {
    /// Retrieve the original pin while disconnecting it from the pwm
    pub fn into_mode<N: PinMode + ValidPinMode<G> + NonPwmPinMode>(self) -> Pin<G, N> {
        self.pin.into_mode::<N>()
    }
}

impl Slices {
    /// Free the pwm registers from the pwm hal struct while consuming it.
    pub fn free(self) -> PWM {
        self._pwm
    }

    //     /// Enable multiple slices at the same time to make their counters sync up.
    //     ///
    //     /// You still need to call `slice` to get an actual slice
    //     pub fn enable_simultaneous<S: SliceId>(&mut self, bits: u8) {
    //         // Enable all slices at the same time
    //         unsafe {
    //             &(*pac::PWM::ptr())
    //                 .en
    //                 .modify(|r, w| w.bits(((r.bits() as u8) | bits) as u32));
    //         }
    //     }

    // /// Get pwm slice based on gpio pin
    // pub fn borrow_mut_from_pin<
    //     S: SliceId,
    //     C: ChannelId,
    //     G: PinId + BankPinId + ValidPwmOutputPin<S, C>,
    //     PM: PinMode + ValidPinMode<G>,
    //     SM: SliceMode + ValidSliceMode<S>,
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
pub struct Channel<S: SliceId, M: SliceMode, C: ChannelId> {
    regs: Registers<S>,
    slice_mode: PhantomData<M>,
    channel_id: PhantomData<C>,
    duty_cycle: u16,
}

impl<S: SliceId, M: SliceMode, C: ChannelId> Channel<S, M, C> {
    pub(super) unsafe fn new(duty_cycle: u16) -> Self {
        Channel {
            regs: Registers::new(),
            slice_mode: PhantomData,
            channel_id: PhantomData,
            duty_cycle,
        }
    }
}

impl<S: SliceId, M: SliceMode, C: ChannelId> Sealed for Channel<S, M, C> {}

impl<S: SliceId, M: SliceMode> PwmPin for Channel<S, M, A> {
    type Duty = u16;

    /// We cant disable the channel without disturbing the other channel.
    /// So this just sets the duty cycle to zero
    fn disable(&mut self) {
        self.duty_cycle = self.regs.read_cc_a();
        self.regs.write_cc_a(0)
    }

    fn enable(&mut self) {
        self.regs.write_cc_a(self.duty_cycle)
    }

    fn get_duty(&self) -> Self::Duty {
        self.regs.read_cc_a()
    }

    fn get_max_duty(&self) -> Self::Duty {
        self.regs.read_top()
    }

    fn set_duty(&mut self, duty: Self::Duty) {
        self.regs.write_cc_a(duty)
    }
}

impl<S: SliceId, M: SliceMode> PwmPin for Channel<S, M, B> {
    type Duty = u16;

    /// We cant disable the channel without disturbing the other channel.
    /// So this just sets the duty cycle to zero
    fn disable(&mut self) {
        self.duty_cycle = self.regs.read_cc_b();
        self.regs.write_cc_b(0)
    }

    fn enable(&mut self) {
        self.regs.write_cc_b(self.duty_cycle)
    }

    fn get_duty(&self) -> Self::Duty {
        self.regs.read_cc_b()
    }

    fn get_max_duty(&self) -> Self::Duty {
        self.regs.read_top()
    }

    fn set_duty(&mut self, duty: Self::Duty) {
        self.regs.write_cc_b(duty)
    }
}

impl<S: SliceId, M: SliceMode + ValidSliceMode<S>> Channel<S, M, A> {
    /// Capture a gpio pin and use it as pwm output for channel A
    pub fn output_to<
        G: PinId + BankPinId + ValidPwmOutputPin<S, A>,
        PM: PinMode + ValidPinMode<G>,
    >(
        &mut self,
        pin: Pin<G, PM>,
    ) -> PwmPinToken<G> {
        PwmPinToken {
            pin: pin.into_mode(),
        }
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

impl<S: SliceId, M: SliceMode + ValidSliceMode<S>> Channel<S, M, B> {
    /// Capture a gpio pin and use it as pwm output for channel B
    pub fn output_to<
        G: PinId + BankPinId + ValidPwmOutputPin<S, B>,
        PM: PinMode + ValidPinMode<G>,
    >(
        &mut self,
        pin: Pin<G, PM>,
    ) -> PwmPinToken<G> {
        PwmPinToken {
            pin: pin.into_mode(),
        }
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

impl<S: SliceId, M: SliceMode + ValidSliceInputMode<S>> Channel<S, M, B> {
    /// Capture a gpio pin and use it as pwm input for channel B
    pub fn input_from<G: PinId + BankPinId + ValidPwmInputPin<S>, PM: PinMode + ValidPinMode<G>>(
        &mut self,
        pin: Pin<G, PM>,
    ) -> PwmPinToken<G> {
        PwmPinToken {
            pin: pin.into_mode(),
        }
    }
}

impl<S: SliceId, M: SliceMode + ValidSliceMode<S>> Slice<S, M> {
    /// Capture a gpio pin and use it as pwm output
    pub fn output_to<
        G: PinId + BankPinId + ValidPwmOutputPin<S, C>,
        PM: PinMode + ValidPinMode<G>,
        C: ChannelId,
    >(
        &mut self,
        pin: Pin<G, PM>,
    ) -> PwmPinToken<G> {
        PwmPinToken {
            pin: pin.into_mode(),
        }
    }
}

impl<S: SliceId, M: SliceMode + ValidSliceInputMode<S>> Slice<S, M> {
    /// Capture a gpio pin and use it as pwm input for channel B
    pub fn input_from<G: PinId + BankPinId + ValidPwmInputPin<S>, PM: PinMode + ValidPinMode<G>>(
        &mut self,
        pin: Pin<G, PM>,
    ) -> PwmPinToken<G> {
        PwmPinToken {
            pin: pin.into_mode(),
        }
    }
}
