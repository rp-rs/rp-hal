//! Pulse Width Modulation (PWM)
//!
//! To access the PWM pins you must instantiate the `Pwm` type, by giving it ownership of the
//! rp2040_pac::PWM intance retrieved via `rp2040_pac::Peripherals::take()`:
//!
//! ```rust
//! use rp2040_hal:: {
//!     gpio::{pin::*, Pins},
//!     pac,
//!     pwm::{Pwm, PwmChannel, PwmOutput, PwmSlice},
//!     sio::Sio,
//! };
//! let pac = pac::Peripherals::take().unwrap();
//! let pwm = Pwm::new(pac.PWM, &mut pac.RESETS);
//! ```
//!
//! Once you have the `Pwm` instance, you must access individual slices and configure them via
//! `PwmSlice` instances:
//!
//! ```rust
//! let pwm0 = pwm.slice(0); // the indices for slices are 0 based
//! pwm0.default_config();
//! pwm0.set_clkdiv_int(125);
//! pwm0.set_wrap(20000);
//! ```
//!
//! Lastly, for each channel of the slice, you can configure the wrap value for counters via
//! `PwmChannel` intances:
//!
//! ```rust
//! let pwm0a = pwm0.channel(PwmOutput::A);
//! pwm0a.set_level(500);
//! ```
//!
//! In order to use GPIOs as outputs for PWM signals, you must also configure them for the specific
//! function:
//!
//! ```rust
//! let sio = Sio::new(pac.SIO);
//! let pins = Pins::new(
//!     pac.IO_BANK0,
//!     pac.PADS_BANK0,
//!     sio.gpio_bank0,
//!     &mut pac.RESETS,
//! );
//! pins.gpio0.into_mode::<FunctionPwm>(); // GPIO 0 outputs PWM channel 0A
//! ```
//!
//! Lastly, you can enable the PWM slice via either the `PwmSlice` or `PwmChannel` instances:
//!
//! ```rust
//! pwm0.enable();
//! // or
//! pwm0a.enable();
//! ```

use super::*;
use crate::resets::SubsystemReset;

/// A PWM channel (i.e. slice output) is represented here.
pub enum PwmOutput {
    /// The A channel of the slice
    A,
    /// The B channel of the slice
    B,
}

/// PWM clock divider modes are represented here.
pub enum PwmClkdivMode {
    /// Free-running counting at rate dictated by fractional divider.
    FreeRunning,
    /// Fractional divider is gated by the PWM B pin.
    BHigh,
    /// Fractional divider advances with each rising edge of the PWM B pin.
    BRising,
    /// Fractional divider advances with each falling edge of the PWM B pin.
    BFalling,
}

/// The `Pwm` type respresents an abstraction of the PWM hardware block.
pub struct Pwm {
    regs: pac::PWM,
}

impl Pwm {
    /// Generate a new `Pwm` instance which takes ownersip of the provided `rp2040_pac::PWM`
    /// object.
    pub fn new(pac_pwm: pac::PWM, resets: &mut pac::RESETS) -> Self {
        pac_pwm.reset_bring_up(resets);
        Pwm { regs: pac_pwm }
    }

    /// Generate a `PwmSlice` instance if the provided slice index is valid. Note that the index is
    /// 0 based.
    pub fn slice(&self, idx: usize) -> Option<PwmSlice> {
        if idx > 7 {
            // The rp2040 PWM block has only 8 slices indexed from 0 to 7
            return None;
        }

        Some(PwmSlice::new(self, idx))
    }
}

/// The `PwmSlice` type represents an abstraction of a PWM slice of the rp2040 PWM block.
pub struct PwmSlice<'a> {
    pwm: &'a Pwm,
    idx: usize,
}

impl<'a> PwmSlice<'a> {
    fn new(pwm: &'a Pwm, idx: usize) -> Self {
        PwmSlice { pwm: pwm, idx: idx }
    }

    /// Disable the phase correct mode of the PWM slice.
    #[inline(always)]
    pub fn disable_phase_correct(&self) {
        self.pwm.regs.ch[self.idx]
            .csr
            .write(|w| w.ph_correct().clear_bit());
    }

    /// Enable the phase correct mode of the PWM slice.
    #[inline(always)]
    pub fn enable_phase_correct(&self) {
        self.pwm.regs.ch[self.idx]
            .csr
            .write(|w| w.ph_correct().set_bit());
    }

    /// Set the integer part of the clock divider for the PWM slice.
    #[inline(always)]
    pub fn set_clkdiv_int(&self, value: u8) {
        self.set_clkdiv_int_frac(value, 0);
    }

    /// Set the fractional part of the clock divider for the PWM slice.
    #[inline(always)]
    pub fn set_clkdiv_int_frac(&self, int: u8, frac: u8) {
        self.pwm.regs.ch[self.idx].div.write(|w| unsafe {
            w.int().bits(int);
            w.frac().bits(frac)
        });
    }

    /// Configure the clock divider mode for the PWM slice.
    #[inline(always)]
    pub fn set_clkdiv_mode(&self, mode: &PwmClkdivMode) {
        match mode {
            PwmClkdivMode::FreeRunning => {
                self.pwm.regs.ch[self.idx].csr.write(|w| w.divmode().div())
            }
            PwmClkdivMode::BHigh => self.pwm.regs.ch[self.idx]
                .csr
                .write(|w| w.divmode().level()),
            PwmClkdivMode::BRising => self.pwm.regs.ch[self.idx].csr.write(|w| w.divmode().rise()),
            PwmClkdivMode::BFalling => self.pwm.regs.ch[self.idx].csr.write(|w| w.divmode().fall()),
        };
    }

    /// Set the value at which the counter wraps back to 0 for the PWM slice.
    #[inline(always)]
    pub fn set_wrap(&self, value: u16) {
        self.pwm.regs.ch[self.idx]
            .top
            .write(|w| unsafe { w.top().bits(value) });
    }

    /// Get the value at which the counter wraps back to 0 for the PWM slice.
    #[inline(always)]
    pub fn get_wrap(&self) -> u16 {
        self.pwm.regs.ch[self.idx].top.read().top().bits()
    }

    /// Enable inversion for the specified output channel of the PWM slice.
    #[inline(always)]
    pub fn enable_output_polarity(&self, output: &PwmOutput) {
        match output {
            PwmOutput::A => self.pwm.regs.ch[self.idx]
                .csr
                .write(|w| w.a_inv().set_bit()),
            PwmOutput::B => self.pwm.regs.ch[self.idx]
                .csr
                .write(|w| w.b_inv().set_bit()),
        }
    }

    /// Disable inversion for the specified output channel of the PWM slice.
    #[inline(always)]
    pub fn disable_output_polarity(&self, output: &PwmOutput) {
        match output {
            PwmOutput::A => self.pwm.regs.ch[self.idx]
                .csr
                .write(|w| w.a_inv().clear_bit()),
            PwmOutput::B => self.pwm.regs.ch[self.idx]
                .csr
                .write(|w| w.b_inv().clear_bit()),
        }
    }

    /// Set the counter compare value for the specified output channel of the slice.
    #[inline(always)]
    pub fn set_chan_level(&self, output: &PwmOutput, value: u16) {
        match output {
            PwmOutput::A => self.pwm.regs.ch[self.idx]
                .cc
                .write(|w| unsafe { w.a().bits(value) }),
            PwmOutput::B => self.pwm.regs.ch[self.idx]
                .cc
                .write(|w| unsafe { w.b().bits(value) }),
        }
    }

    /// Get the counter compare value for the specified output channel of the slice.
    #[inline(always)]
    pub fn get_chan_level(&self, output: &PwmOutput) -> u16 {
        match output {
            PwmOutput::A => self.pwm.regs.ch[self.idx].cc.read().a().bits(),
            PwmOutput::B => self.pwm.regs.ch[self.idx].cc.read().b().bits(),
        }
    }

    /// Enable the PWM slice. This enables both channels of the slice.
    #[inline(always)]
    pub fn enable(&self) -> () {
        self.pwm.regs.ch[self.idx].csr.write(|w| w.en().set_bit());
    }

    /// Disable the PWM slice. This disables both channels of the slice.
    #[inline(always)]
    pub fn disable(&self) -> () {
        self.pwm.regs.ch[self.idx].csr.write(|w| w.en().clear_bit());
    }

    /// Applies a default configuration for the PWM slice.
    pub fn default_config(&self) {
        self.disable_phase_correct();
        self.set_clkdiv_int(1);
        self.set_clkdiv_mode(&PwmClkdivMode::FreeRunning);
        self.disable_output_polarity(&PwmOutput::A);
        self.disable_output_polarity(&PwmOutput::B);
        self.set_wrap(0xffff);
    }

    /// Generate a `PwmChannel` instance for the specified output of the slice.
    pub fn channel(&'a self, output: PwmOutput) -> PwmChannel {
        PwmChannel::new(self, output)
    }
}

/// The `PwmChannel` type respresents an abstraction for one of the PWM channels of each slice in
/// the rp2040 hardware.
pub struct PwmChannel<'a> {
    slice: &'a PwmSlice<'a>,
    output: PwmOutput,
}

impl<'a> PwmChannel<'a> {
    fn new(slice: &'a PwmSlice<'a>, output: PwmOutput) -> Self {
        PwmChannel {
            slice: slice,
            output: output,
        }
    }

    /// Set the counter compare value for the channel.
    #[inline(always)]
    pub fn set_level(&self, level: u16) {
        self.slice.set_chan_level(&self.output, level);
    }

    /// Get the counter compare value for the channel.
    #[inline(always)]
    pub fn get_level(&self) -> u16 {
        self.slice.get_chan_level(&self.output)
    }

    /// Enable inversion for the channel.
    #[inline(always)]
    pub fn enable_polarity(&self) {
        self.slice.enable_output_polarity(&self.output);
    }

    /// Disable inversion for the channel.
    #[inline(always)]
    pub fn disable_polarity(&self) {
        self.slice.disable_output_polarity(&self.output);
    }
}

impl<'a> embedded_hal::PwmPin for PwmChannel<'a> {
    /// Type for the `duty` methods
    ///
    /// The representation is similar to the pico-sdk, namely a 16bit unsigned integer (i.e. `0 .. 65535`)
    type Duty = u16;

    /// Disable the underlying slice associated with the PWM channel. Channels cannot be
    /// individually disabled on the rp2040.
    fn disable(&mut self) {
        self.slice.disable();
    }

    /// Enable the underlying slice associated with the PWM channel. Channels cannot be
    /// individually enabled on the rp2040.
    fn enable(&mut self) {
        self.slice.enable();
    }

    /// Get the counter compare value for the channel.
    fn get_duty(&self) -> Self::Duty {
        self.slice.get_chan_level(&self.output)
    }

    /// Get the value at which the counter wraps back to 0 for the channel.
    fn get_max_duty(&self) -> Self::Duty {
        self.slice.get_wrap()
    }

    /// Set the counter compare value for the channel.
    fn set_duty(&mut self, duty: Self::Duty) {
        self.set_level(duty);
    }
}
