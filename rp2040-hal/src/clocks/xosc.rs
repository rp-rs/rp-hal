use core::marker::PhantomData;
use embedded_time::duration::*;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::*;
use num::rational::Ratio;
use num::{CheckedDiv, CheckedMul};

mod sealed {
    pub trait Sealed {}
}

pub trait State: sealed::Sealed {}
pub trait EnabledOrStable: State {}

pub struct Disabled;
pub struct Enabled;
pub struct Stable;

impl State for Disabled {}
impl State for Enabled {}
impl State for Stable {}

impl EnabledOrStable for Enabled {}
impl EnabledOrStable for Stable {}

impl sealed::Sealed for Disabled {}
impl sealed::Sealed for Enabled {}
impl sealed::Sealed for Stable {}

#[derive(Debug)]
pub enum Error {
    StartUpDelayOverflows,
    RateOutOfRange,
}

/// [Chapter 2 Section 16](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details
///
/// # Example
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
/// let stable_xosc = loop {
///     xosc = match xosc.stable() {
///         Ok(x) => break x,
///         Err(x) => x,
///     };
/// };
///
/// // Do things with the clock source
/// ```
pub struct XOsc<S: State, R: Rate> {
    inner: pac::XOSC,
    rate: R,
    state: PhantomData<S>,
}

impl<R> XOsc<Disabled, R>
where
    R: Rate + FixedPoint<T = u32> + Copy + PartialOrd<Megahertz>,
    Megahertz: PartialOrd<R>,
{
    /// Create a new wrapper for an oscillator with a given frequency.
    pub fn new(peri: pac::XOSC, rate: R) -> Result<Self, Error> {
        let mut dev = XOsc {
            inner: peri,
            rate,
            state: PhantomData,
        };

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

        XOsc {
            inner: self.inner,
            rate: self.rate,
            state: PhantomData,
        }
    }

    /// Release the inner peripheral.
    pub fn free(self) -> pac::XOSC {
        self.inner
    }
}

impl<R: Rate> XOsc<Enabled, R> {
    /// Check if the oscillator is stable and can be used. Returns the old state if is not yet stable.
    pub fn stable(self) -> Result<XOsc<Stable, R>, Self> {
        if !self.inner.status.read().stable().bit_is_set() {
            return Err(self);
        }

        Ok(XOsc {
            inner: self.inner,
            rate: self.rate,
            state: PhantomData,
        })
    }
}

impl<R: Rate, S: EnabledOrStable> XOsc<S, R> {
    /// This disables the device without checking if the clock is still in use
    // ToDo: Should this be `unsafe`?
    pub fn disable_unchecked(self) -> XOsc<Disabled, R> {
        self.inner.ctrl.write(|w| w.enable().disable());

        XOsc {
            inner: self.inner,
            rate: self.rate,
            state: PhantomData,
        }
    }
}

fn delay_cnt<R: Rate + FixedPoint<T = u32>, D: Duration + FixedPoint<T = u32>>(
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

    if reg_val > 0x3fff {
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
