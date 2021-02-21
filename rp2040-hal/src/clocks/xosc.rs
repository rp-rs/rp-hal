use super::Rate;
use crate::clocks::{ClkDevice, ClkSource};
use core::convert::TryFrom;
use embedded_time::rate::{Extensions, Megahertz};
use embedded_time::{duration::*, fixed_point::FixedPoint};
use nb::Error::{Other, WouldBlock};
use num::{rational::Ratio, CheckedDiv, CheckedMul};

mod sealed {
    pub trait Sealed {}
}

pub trait State: sealed::Sealed {}
pub trait EnabledOrStable: State {}

pub struct Disabled;
pub struct Enabled {
    token: Option<StableToken>,
}
pub struct Stable;
pub struct Shared {
    outstanding_shares: u32,
}

impl State for Disabled {}
impl State for Enabled {}
impl State for Stable {}
impl State for Shared {}

impl EnabledOrStable for Enabled {}
impl EnabledOrStable for Stable {}

impl sealed::Sealed for Disabled {}
impl sealed::Sealed for Enabled {}
impl sealed::Sealed for Stable {}
impl sealed::Sealed for Shared {}

pub struct StableToken {
    _private: (),
}

#[derive(Debug)]
pub enum Error {
    StartUpDelayOverflows,
    RateOutOfRange,
}

#[derive(Debug)]
pub struct ErrorTokenAlreadyTaken;

/// [Chapter 2 Section 16](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
///
/// # Example
/// ## Blocking
/// ```rust no_run
/// use embedded_time::rate::Extensions as _;
///
/// use rp2040_pac::Peripherals;
/// use rp2040_hal::clocks::XOsc;
///
/// let p = Peripherals::take().unwrap();
///
/// let xosc = XOsc::new_stable_blocking(p.XOSC, 12.MHz().into()).unwrap();
///
/// // Do things with the clock source
/// ```
///
/// ## Non-Blocking
/// ```rust no_run
/// use embedded_time::rate::Extensions as _;
/// use embedded_time::duration::Extensions as _;
///
/// use rp2040_pac::Peripherals;
/// use rp2040_hal::clocks::XOsc;
///
/// let p = Peripherals::take().unwrap();
///
/// let mut xosc = XOsc::new(p.XOSC, 12.MHz().into()).unwrap();
///
/// // Optional, defaults to 1ms
/// xosc.set_startup_delay(&42.milliseconds()).unwrap();
///
/// let mut xosc = xosc.enable();
/// let token = nb::block!(xosc.await_stable()).unwrap();
/// let xosc = xosc.stable(token);
///
/// // Do things with the clock source
/// ```
pub struct XOsc<S: State> {
    inner: pac::XOSC,
    rate: Rate,
    state: S,
}
pub type StableXOsc = XOsc<Stable>;

impl ClkDevice for StableXOsc {}
impl ClkSource for &StableXOsc {
    type DeviceName = pac::XOSC;

    fn rate(&self) -> Rate {
        self.rate
    }
}

impl XOsc<Stable> {
    /// Convenient method for getting a XOsc without caring about the state transitions.
    pub fn new_stable_blocking(peri: pac::XOSC, rate: Rate) -> Result<StableXOsc, Error> {
        let mut enabled = XOsc::<Disabled>::new(peri, rate)?.enable();
        let token = nb::block!(enabled.await_stable()).unwrap();
        Ok(enabled.stable(token))
    }
}

/// # Disabled
impl XOsc<Disabled> {
    /// Create a new wrapper for an oscillator with a given frequency.
    pub fn new(peri: pac::XOSC, rate: Rate) -> Result<Self, Error> {
        let mut dev = XOsc {
            inner: peri,
            rate,
            state: Disabled {},
        };
        dev.raw_disable();

        match Megahertz::<u32>::try_from(rate) {
            Ok(val) if (1u32.MHz()..=15u32.MHz()).contains(&val) => (),
            _ => return Err(Error::RateOutOfRange),
        }

        dev.inner.ctrl.write(|w| w.freq_range()._1_15mhz());

        dev.set_startup_delay(&Milliseconds(1))?;

        Ok(dev)
    }

    /// Sets the startup delay for the osc. Defaults to 1ms
    pub fn set_startup_delay<D: Duration + FixedPoint<T = u32>>(
        &mut self,
        duration: &D,
    ) -> Result<(), Error> {
        let startup_delay = delay_cnt(&self.rate, duration).ok_or(Error::StartUpDelayOverflows)?;

        self.inner
            .startup
            .write(|w| unsafe { w.delay().bits(startup_delay) });

        Ok(())
    }

    /// Enable the peripheral
    pub fn enable(self) -> XOsc<Enabled> {
        self.inner.ctrl.write(|w| w.enable().enable());

        self.transition(Enabled {
            token: Some(StableToken { _private: () }),
        })
    }

    /// Release the inner peripheral.
    pub fn free(self) -> pac::XOSC {
        self.inner
    }
}

/// # Enabled
impl XOsc<Enabled> {
    /// Transition to the stable state. Acquire a token by calling `await_stable`
    pub fn stable(self, _: StableToken) -> StableXOsc {
        self.transition(Stable {})
    }

    /// Get a nb result to await stable
    pub fn await_stable(&mut self) -> nb::Result<StableToken, ErrorTokenAlreadyTaken> {
        let is_stable = self.inner.status.read().stable().bit_is_set();
        if !is_stable {
            return Err(WouldBlock);
        }

        match self.state.token.take() {
            None => Err(Other(ErrorTokenAlreadyTaken {})),
            Some(token) => Ok(token),
        }
    }

    /// Disable the device returning to the Disabled state. Returns an error with the current
    /// state if the stable token was already taken.
    pub fn disable(mut self) -> Result<XOsc<Disabled>, Self> {
        if self.state.token.is_none() {
            return Err(self);
        }

        self.raw_disable();
        Ok(self.transition(Disabled {}))
    }
}

/// # Stable
impl XOsc<Stable> {
    /// This disables the device, no devices should be able to still use the peripheral.
    pub fn disable(mut self) -> XOsc<Disabled> {
        self.raw_disable();
        self.transition(Disabled {})
    }

    /// Lock the peripheral and return a handle that proves that you did so.
    ///
    /// This allows a user to assume that the oscillator won't change during runtime.
    pub fn leak(self) -> LeakedXOsc {
        LeakedXOsc { rate: self.rate }
    }

    /// Transition to sharing the oscillator
    ///
    /// This allows multiple parties to use the device and return the shares when they are done.
    pub fn share(self) -> XOsc<Shared> {
        self.transition(Shared {
            outstanding_shares: 0,
        })
    }
}

/// # Shared
impl XOsc<Shared> {
    /// Take a share of the peripheral.
    pub fn take_share(&mut self) -> XOscShare {
        self.state.outstanding_shares = self
            .state
            .outstanding_shares
            .checked_add(1)
            .expect("Given out over 2^32 xosc shares");
        XOscShare { rate: self.rate }
    }

    /// Return the share to the peripheral
    pub fn return_share(&mut self, _: XOscShare) {
        self.state.outstanding_shares = self
            .state
            .outstanding_shares
            .checked_sub(1)
            .expect("Returned more xosc shares than were given out");
    }

    /// Check whether not all shares have been returned.
    pub fn has_outstanding_shares(&self) -> bool {
        self.state.outstanding_shares > 0
    }

    /// Returns the xosc back to single ownership
    ///
    /// This could be used to deactivate the peripheral.
    ///
    /// # Panics
    /// If there are still outstanding shares. `has_outstanding_shares` can be used to make sure
    /// all shares were returned.
    pub fn un_share(self) -> XOsc<Stable> {
        if self.has_outstanding_shares() {
            panic!(
                "Tried to un-share the XOsc peripheral while there were still outstanding shares!"
            );
        }

        self.transition(Stable {})
    }
}

impl<S: State> XOsc<S> {
    fn transition<To: State>(self, to: To) -> XOsc<To> {
        XOsc {
            inner: self.inner,
            rate: self.rate,
            state: to,
        }
    }

    fn raw_disable(&mut self) {
        self.inner.ctrl.write(|w| w.enable().disable());
    }
}

/// A leaked oscillator, which can never be changed again
///
/// This makes it easy to pass around oscillator without caring about lifetimes. The original
/// peripheral is guaranteed to never be freed, as an instance of this type can only be created
/// by calling `StableXOsc::leak()` which consumes the peripheral.
#[derive(Copy, Clone, Debug)]
pub struct LeakedXOsc {
    rate: Rate,
}
impl ClkSource for LeakedXOsc {
    type DeviceName = pac::XOSC;
    fn rate(&self) -> Rate {
        self.rate
    }
}

/// A shared oscillator, which can be disabled after all shares were returned
///
/// This allows a setup where the oscillator is disabled and enabled multiple times during
/// the runtime.
pub struct XOscShare {
    rate: Rate,
}
impl ClkSource for XOscShare {
    type DeviceName = pac::XOSC;
    fn rate(&self) -> Rate {
        self.rate
    }
}

#[inline(always)]
fn delay_cnt<D: Duration + FixedPoint<T = u32>>(rate: &Rate, startup_delay: &D) -> Option<u16> {
    let f_osc = Ratio::from_integer(*rate.integer());
    let t_stable = Ratio::from_integer(*startup_delay.integer());

    let f_unit = rate.scaling_factor();
    let f_unit = Ratio::new(*f_unit.numerator(), *f_unit.denominator());

    let t_unit = D::SCALING_FACTOR;
    let t_unit = Ratio::new(*t_unit.numerator(), *t_unit.denominator());

    let reg_val = f_unit
        .checked_mul(&t_unit)?
        .checked_mul(&f_osc)?
        .checked_mul(&t_stable)?
        .checked_div(&Ratio::from_integer(256))?
        .ceil()
        .to_integer();

    if reg_val > 0b0011_1111_1111_1111 {
        // The register can only hold 14 bits
        return None;
    }

    Some(reg_val as u16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_time::rate::Extensions;

    #[test]
    fn startup_delay_of_0_should_return_0() {
        assert_eq!(delay_cnt(&12.MHz().into(), &Milliseconds(0)), Some(0));
    }

    #[test]
    fn datasheet_example_matches_impl() {
        let delay = &Milliseconds(1);
        assert_eq!(delay_cnt(&12.MHz().into(), delay), Some(47));
        assert_eq!(delay_cnt(&12_000.kHz().into(), delay), Some(47));
        assert_eq!(delay_cnt(&12_000_000.Hz().into(), delay), Some(47));

        let delay = &Nanoseconds(1_000_000);
        assert_eq!(delay_cnt(&12.MHz().into(), delay), Some(47));
        assert_eq!(delay_cnt(&12_000.kHz().into(), delay), Some(47));
        assert_eq!(delay_cnt(&12_000_000.Hz().into(), delay), Some(47));
    }

    #[test]
    fn high_value_examples() {
        assert_eq!(
            delay_cnt(&15.MHz().into(), &279_603_200.nanoseconds()),
            Some(0x3fff)
        );

        // This should overflow the available 14 bits
        assert_eq!(
            delay_cnt(&15.MHz().into(), &279_603_201.nanoseconds()),
            None
        );
    }
}
