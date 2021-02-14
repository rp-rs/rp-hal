use crate::clocks::{ClkDevice, ClkRate, ClkSource};
use embedded_time::{duration::*, fixed_point::FixedPoint, rate::*};
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

impl State for Disabled {}
impl State for Enabled {}
impl State for Stable {}

impl EnabledOrStable for Enabled {}
impl EnabledOrStable for Stable {}

impl sealed::Sealed for Disabled {}
impl sealed::Sealed for Enabled {}
impl sealed::Sealed for Stable {}

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
/// let xosc = XOsc::new_stable_blocking(p.XOSC, 12.MHz()).unwrap();
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
/// let mut xosc = XOsc::new(p.XOSC, 12.MHz()).unwrap();
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
pub struct XOsc<S: State, R: ClkRate> {
    inner: pac::XOSC,
    rate: R,
    state: S,
}
pub type StableXOsc<R> = XOsc<Stable, R>;

impl<R: ClkRate> ClkDevice for XOsc<Stable, R> {}
impl<R: ClkRate> ClkSource<XOsc<Stable, R>, R> for &XOsc<Stable, R> {
    fn rate(&self) -> R {
        self.rate
    }
}

impl<R> XOsc<Stable, R>
where
    R: ClkRate,
    Megahertz: PartialOrd<R>,
{
    /// Convenient method for getting a XOsc without caring about the state transitions.
    pub fn new_stable_blocking(peri: pac::XOSC, rate: R) -> Result<XOsc<Stable, R>, Error> {
        let mut enabled = XOsc::<Disabled, R>::new(peri, rate)?.enable();
        let token = nb::block!(enabled.await_stable()).unwrap();
        Ok(enabled.stable(token))
    }
}

/// # Disabled
impl<R> XOsc<Disabled, R>
where
    R: ClkRate,
    Megahertz: PartialOrd<R>,
{
    /// Create a new wrapper for an oscillator with a given frequency.
    pub fn new(peri: pac::XOSC, rate: R) -> Result<Self, Error> {
        let mut dev = XOsc {
            inner: peri,
            rate,
            state: Disabled {},
        };
        dev.raw_disable();

        if !(1.MHz()..=15.MHz()).contains(&rate) {
            return Err(Error::RateOutOfRange);
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
    pub fn enable(self) -> XOsc<Enabled, R> {
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
impl<R: ClkRate> XOsc<Enabled, R> {
    /// Transition to the stable state. Acquire a token by calling `await_stable`
    pub fn stable(self, _: StableToken) -> XOsc<Stable, R> {
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
    pub fn disable(mut self) -> Result<XOsc<Disabled, R>, Self> {
        if self.state.token.is_none() {
            return Err(self);
        }

        self.raw_disable();
        Ok(self.transition(Disabled {}))
    }
}

/// # Stable
impl<R: ClkRate> XOsc<Stable, R> {
    /// This disables the device without checking if the clock is still in use
    // ToDo: Should this be `unsafe`?
    pub fn disable_unchecked(mut self) -> XOsc<Disabled, R> {
        self.raw_disable();
        self.transition(Disabled {})
    }
}

impl<S: State, R: ClkRate> XOsc<S, R> {
    fn transition<To: State>(self, to: To) -> XOsc<To, R> {
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

#[inline(always)]
fn delay_cnt<R: ClkRate, D: Duration + FixedPoint<T = u32>>(
    rate: &R,
    startup_delay: &D,
) -> Option<u16> {
    let f_osc = Ratio::from_integer(*rate.integer());
    let t_stable = Ratio::from_integer(*startup_delay.integer());

    let f_unit = R::SCALING_FACTOR;
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

    #[test]
    fn startup_delay_of_0_should_return_0() {
        assert_eq!(delay_cnt(&12.MHz(), &Milliseconds(0)), Some(0));
    }

    #[test]
    fn datasheet_example_matches_impl() {
        let delay = &Milliseconds(1);
        assert_eq!(delay_cnt(&12.MHz(), delay), Some(47));
        assert_eq!(delay_cnt(&12_000.kHz(), delay), Some(47));
        assert_eq!(delay_cnt(&12_000_000.Hz(), delay), Some(47));

        let delay = &Nanoseconds(1_000_000);
        assert_eq!(delay_cnt(&12.MHz(), delay), Some(47));
        assert_eq!(delay_cnt(&12_000.kHz(), delay), Some(47));
        assert_eq!(delay_cnt(&12_000_000.Hz(), delay), Some(47));
    }

    #[test]
    fn high_value_examples() {
        assert_eq!(
            delay_cnt(&15.MHz(), &279_603_200.nanoseconds()),
            Some(0x3fff)
        );

        // This should overflow the available 14 bits
        assert_eq!(delay_cnt(&15.MHz(), &279_603_201.nanoseconds()), None);
    }
}
