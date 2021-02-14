//! PLL wrapper
//!

use super::xosc::StableXOsc;
use super::{ClkRate, ClkSource};
use crate::clocks::ClkDevice;
use core::marker::PhantomData;
use core::ops::Deref;
use embedded_time::rate::*;
use nb::Error::{Other, WouldBlock};
use pac::pll_sys::RegisterBlock;
use state::*;

mod state {
    use crate::clocks::pll::PllDev;

    pub trait State {}

    pub struct Disabled(pub(super) super::PllParams);
    pub struct Enabled<P: PllDev> {
        pub(super) token: Option<super::StableToken<P>>,
    }
    pub struct Stable;

    impl State for Disabled {}
    impl<P: PllDev> State for Enabled<P> {}
    impl State for Stable {}
}

/// All the devices that can be used with this implementation have to implement this marker trait;
pub trait PllDev: Deref<Target = RegisterBlock> {
    /// The maximum rate this PLL can output according to the datasheet.
    const MAX_OUT_RATE: Megahertz<u32>;
}

impl PllDev for pac::PLL_SYS {
    const MAX_OUT_RATE: Megahertz<u32> = Megahertz(133);
}

impl PllDev for pac::PLL_USB {
    const MAX_OUT_RATE: Megahertz<u32> = Megahertz(48);
}

/// A list of contraints for the PLL according to datasheet section 2.18.2
pub mod constraints {
    use core::ops::RangeInclusive;
    use embedded_time::rate::Hertz;
    use embedded_time::rate::Megahertz;

    /// Minimum reference clock after reference divider
    pub const MIN_REF_CLK: Megahertz<u32> = Megahertz(5);

    /// Allowed range for the voltage controlled oscillator (VCO)
    pub const F_OUT_VCO_RANGE: RangeInclusive<Hertz<u32>> =
        Hertz(400_000_000)..=Hertz(1_600_000_000);

    /// Allowed range for the feedback divider
    pub const FB_DIV_RANGE: RangeInclusive<u16> = 16..=320;

    /// Allowed range for both of the post dividers
    pub const POST_DIV_RANGE: RangeInclusive<u8> = 1..=7;
}

/// Returned if the requested rate exceeds the ratings given in the datasheet.
#[derive(Debug)]
pub struct ParameterOutOfRangeError;

/// Error returned if the device should be disabled but the token is already gone.
/// This way it is not possible to have more than one token at any point in time.
#[derive(Debug)]
pub struct ErrorTokenAlreadyTaken;

/// A token that represents that the Pll has locked.
pub struct StableToken<P: PllDev> {
    _dev: PhantomData<P>,
}

/// Pll wrapper
///
/// # Example
/// # Blocking
/// ```rust no_run
/// use embedded_time::rate::Extensions as _;
///
/// use rp2040_pac::Peripherals;
/// use rp2040_hal::clocks::XOsc;
/// use rp2040_hal::clocks::pll::Pll;
///
/// let p = Peripherals::take().unwrap();
///
/// let xosc = XOsc::new_stable_blocking(p.XOSC, 12.MHz()).unwrap();
///
/// let pll = Pll::new_stable_blocking(p.PLL_SYS, &xosc, 125.MHz()).unwrap();
///
/// // Do things with the clock source
/// ```
///
/// ## Non-Blocking
/// ```rust no_run
/// use embedded_time::rate::Extensions as _;
///
/// use rp2040_pac::Peripherals;
/// use rp2040_hal::clocks::XOsc;
/// use rp2040_hal::clocks::pll::Pll;
///
/// let p = Peripherals::take().unwrap();
///
/// let xosc = XOsc::new_stable_blocking(p.XOSC, 12.MHz()).unwrap();
///
/// let pll = Pll::new(p.PLL_SYS, &xosc, 125.MHz()).unwrap();
///
/// let mut pll = pll.enable();
/// let token = nb::block!(pll.await_stable()).unwrap();
/// let pll = pll.stable(token);
///
/// // Do things with the clock source
/// ```
///
pub struct Pll<Dev: PllDev, Src: ClkSource<StableXOsc<R>, R>, S: State, R: ClkRate> {
    dev: Dev,
    src: Src,
    state: S,
    _rate: PhantomData<R>,
}

/// Convenience type for a stable PLL
pub type StablePll<Dev, Src, R> = Pll<Dev, Src, Stable, R>;

impl<R: ClkRate, Dev: PllDev, Src: ClkSource<StableXOsc<R>, R>> ClkDevice
    for Pll<Dev, Src, Stable, R>
{
}
impl<R: ClkRate, Dev: PllDev, Src: ClkSource<StableXOsc<R>, R>>
    ClkSource<Pll<Dev, Src, Stable, R>, R> for Pll<Dev, Src, Stable, R>
{
    fn rate(&self) -> R {
        self.params()
            .resulting_rate(self.src.rate().as_hz().unwrap())
            .into()
    }
}

impl<Dev: PllDev, Src: ClkSource<StableXOsc<R>, R>, R: ClkRate> Pll<Dev, Src, Disabled, R> {
    /// Create a new Pll
    pub fn new(dev: Dev, src: Src, output: R) -> Result<Self, ParameterOutOfRangeError> {
        let input = src.rate().as_hz().ok_or(ParameterOutOfRangeError {})?;
        let output = output.as_hz().ok_or(ParameterOutOfRangeError {})?;

        let params = PllParams::find(input, output, None, OptimizeFor::LowJitter)
            .ok_or(ParameterOutOfRangeError {})?;

        if params.resulting_rate(input) > Dev::MAX_OUT_RATE {
            return Err(ParameterOutOfRangeError {});
        }

        Ok(Pll {
            dev,
            src,
            state: Disabled(params),
            _rate: PhantomData,
        })
    }

    /// Convenience method for getting a running PLL without needing to worry about the state
    /// machine.
    pub fn new_stable_blocking(
        dev: Dev,
        src: Src,
        output: R,
    ) -> Result<Pll<Dev, Src, Stable, R>, ParameterOutOfRangeError> {
        let mut pll = Self::new(dev, src, output)?.enable();
        let token = nb::block!(pll.await_stable()).unwrap();
        Ok(pll.stable(token))
    }

    /// Set all the dividers based of the struct
    fn set_params(&mut self, params: PllParams) {
        // Safety: PllParams can only contain valid values for these parameters
        unsafe {
            self.dev.cs.modify(|_, w| w.refdiv().bits(params.ref_div));
            self.dev
                .fbdiv_int
                .modify(|_, w| w.fbdiv_int().bits(params.fb_div));
            self.dev.prim.modify(|_, w| {
                w.postdiv1()
                    .bits(params.post_div1)
                    .postdiv2()
                    .bits(params.post_div2)
            });
        }
    }

    /// Set the requested and enable the PLL. Before the PLL can be used for anything is needs to
    /// be stable.
    pub fn enable(mut self) -> Pll<Dev, Src, Enabled<Dev>, R> {
        // Turn of the power, it it may have been enabled before
        self.raw_disable_pwr();

        self.set_params(self.state.0);

        // Enable power
        self.dev
            .pwr
            .modify(|_, w| w.pd().clear_bit().vcopd().clear_bit());

        // The VCO is now running and we need to wait until the Pll is locked, to call is stable
        self.transition(Enabled {
            token: Some(StableToken { _dev: PhantomData }),
        })
    }

    /// Destructure the PLL back into its parts
    pub fn free(self) -> (Dev, Src) {
        (self.dev, self.src)
    }
}

impl<Dev: PllDev, Src: ClkSource<StableXOsc<R>, R>, R: ClkRate> Pll<Dev, Src, Enabled<Dev>, R> {
    /// Transition to the stable state. Obtain a token by calling `await_stable`.
    pub fn stable(self, _: StableToken<Dev>) -> Pll<Dev, Src, Stable, R> {
        // Enable power to the post dividers
        self.dev.pwr.modify(|_, w| w.postdivpd().clear_bit());

        self.transition(Stable {})
    }

    /// Check if the PLL is stable. Meaning the `locked` flag is set in the `CS` register.
    pub fn await_stable(&mut self) -> nb::Result<StableToken<Dev>, ErrorTokenAlreadyTaken> {
        let is_stable = self.dev.cs.read().lock().bit_is_set();
        if !is_stable {
            return Err(WouldBlock);
        }

        match self.state.token.take() {
            None => Err(Other(ErrorTokenAlreadyTaken {})),
            Some(token) => Ok(token),
        }
    }

    /// Disable the PLL, this can be useful if you can not get the PLL to lock and want to change
    /// settings to try again.
    pub fn disable(mut self) -> Result<Pll<Dev, Src, Disabled, R>, Self> {
        if self.state.token.is_none() {
            return Err(self);
        }

        self.raw_disable_pwr();
        let next = Disabled(self.params());
        Ok(self.transition(next))
    }
}

impl<Dev: PllDev, Src: ClkSource<StableXOsc<R>, R>, R: ClkRate> Pll<Dev, Src, Stable, R> {
    /// Disables the PLL without checking if the output clock is still in use.
    pub fn disable_unchecked(mut self) -> Result<Pll<Dev, Src, Disabled, R>, Self> {
        self.raw_disable_pwr();
        let next = Disabled(self.params());
        Ok(self.transition(next))
    }
}

impl<D: PllDev, Sr: ClkSource<StableXOsc<R>, R>, St: State, R: ClkRate> Pll<D, Sr, St, R> {
    fn transition<To: State>(self, to: To) -> Pll<D, Sr, To, R> {
        Pll {
            dev: self.dev,
            src: self.src,
            state: to,
            _rate: self._rate,
        }
    }

    fn raw_disable_pwr(&mut self) {
        self.dev.pwr.reset();
    }

    fn params(&self) -> PllParams {
        PllParams {
            ref_div: self.dev.cs.read().refdiv().bits(),
            fb_div: self.dev.fbdiv_int.read().fbdiv_int().bits(),
            post_div1: self.dev.prim.read().postdiv1().bits(),
            post_div2: self.dev.prim.read().postdiv2().bits(),
        }
    }
}

/// What to aim for when choosing PLL parameters.
pub enum OptimizeFor {
    /// Tries to use a low VCO rate in order to use as little power as possible.
    LowPower,

    /// Tries to use a high VCO rate in order to have as little clock jitter as possible.
    LowJitter,
}

/// A set of parameters to set up the PLL
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub struct PllParams {
    ref_div: u8,
    fb_div: u16,
    post_div1: u8,
    post_div2: u8,
}

impl PllParams {
    /// Calculate the output rate for a pll configured with this set of parameters and the given
    /// `input` as reference clock.
    pub fn resulting_rate(&self, input: Hertz) -> Hertz {
        input * (self.fb_div as u32)
            / (self.ref_div as u32)
            / (self.post_div1 as u32)
            / (self.post_div2 as u32)
    }

    /// Find a set of settings for the PLL to match the given output.
    ///
    /// The calculation is based on [the script given in the datasheet](https://github.com/raspberrypi/pico-sdk/tree/master/src/rp2_common/hardware_clocks/scripts/vcocalc.py#L1-L37)
    /// (section 2.18.2).
    ///
    /// - If `low_vco` is true a more power efficient set is chosen, but the output jitter might be higher.
    /// - If `vco_max` is Some() then no VCO higher than that will be used.
    ///
    /// # TODO
    /// This function could be `const` since we only loop and do basic arithmatic. But it does get
    /// more difficult to read if written without iterators.
    ///
    /// # Example
    /// ```
    /// use rp2040_hal::clocks::pll::{PllParams, OptimizeFor};
    /// use embedded_time::rate::Extensions;
    ///
    /// // Get a low power set of setting for about 125 MHz
    /// let params = PllParams::find(12_000_000.Hz(), 125_000_000.Hz(), Some(600_000_000.Hz()), OptimizeFor::LowPower).unwrap();
    /// assert_eq!(params.ref_div(), 1);
    /// assert_eq!(params.fb_div(), 42);
    /// assert_eq!(params.post_div1(), 4);
    /// assert_eq!(params.post_div2(), 1);
    ///
    /// assert_eq!(params.resulting_rate(12_000_000.Hz()), 126_000_000u32.Hz());
    /// ```
    pub fn find(
        input: Hertz,
        output: Hertz,
        vco_max: Option<Hertz>,
        target: OptimizeFor,
    ) -> Option<Self> {
        use constraints::*;
        let post_divs = POST_DIV_RANGE
            .into_iter()
            .flat_map(|pd2| POST_DIV_RANGE.into_iter().map(move |pd1| (pd2, pd1)));

        let mut fb_iter = FB_DIV_RANGE;
        let fb_divs = core::iter::from_fn(|| match target {
            OptimizeFor::LowPower => fb_iter.next(),
            OptimizeFor::LowJitter => fb_iter.next_back(),
        });

        let vco_range = *F_OUT_VCO_RANGE.start()..=vco_max.unwrap_or(*F_OUT_VCO_RANGE.end());

        let (_margin, (_vco, fb_div, post_div2, post_div1)) = fb_divs
            .map(|fb_div| (input * (fb_div as u32), fb_div))
            .filter(|(vco, _fb_div)| vco_range.contains(vco))
            .flat_map(|(vco, fb_div)| {
                post_divs
                    .clone()
                    .map(move |(pd2, pd1)| (vco, fb_div, pd2, pd1))
            })
            .map(|params| {
                let (vco, _fb_div, pd2, pd1) = params;
                let out = vco / pd1 as u32 / pd2 as u32;
                let margin = if output > out {
                    output - out
                } else {
                    out - output
                };
                (margin, params)
            })
            .min_by_key(|(margin, _)| *margin)?;

        Some(PllParams {
            ref_div: 1,
            fb_div,
            post_div1,
            post_div2,
        })
    }

    /// Get the value for the reference divider
    pub fn ref_div(&self) -> u8 {
        self.ref_div
    }

    /// Get the value for the feedback divider
    pub fn fb_div(&self) -> u16 {
        self.fb_div
    }

    /// Get the value of the first post divider
    pub fn post_div1(&self) -> u8 {
        self.post_div1
    }

    /// Get the value od the second post divider
    pub fn post_div2(&self) -> u8 {
        self.post_div2
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn datsheet_example() {
        assert_eq!(
            PllParams::find(
                12_000_000.Hz(),
                48_000_000.Hz(),
                None,
                OptimizeFor::LowJitter
            )
            .unwrap(),
            PllParams {
                ref_div: 1,
                fb_div: 120,
                post_div1: 6,
                post_div2: 5,
            }
        );
    }

    #[test]
    fn datsheet_example_low_vco() {
        assert_eq!(
            PllParams::find(
                12_000_000.Hz(),
                48_000_000.Hz(),
                None,
                OptimizeFor::LowPower
            )
            .unwrap(),
            PllParams {
                ref_div: 1,
                fb_div: 36,
                post_div1: 3,
                post_div2: 3,
            }
        );
    }

    #[test]
    fn datsheet_example_limited_vco() {
        assert_eq!(
            PllParams::find(
                12_000_000.Hz(),
                125_000_000.Hz(),
                None,
                OptimizeFor::LowPower
            )
            .unwrap(),
            PllParams {
                ref_div: 1,
                fb_div: 125,
                post_div1: 6,
                post_div2: 2,
            }
        );

        let expected = PllParams {
            ref_div: 1,
            fb_div: 42,
            post_div1: 4,
            post_div2: 1,
        };
        assert_eq!(
            PllParams::find(
                12_000_000.Hz(),
                125_000_000.Hz(),
                Some(600_000_000.Hz()),
                OptimizeFor::LowPower
            )
            .unwrap(),
            expected
        );

        assert_eq!(
            expected.resulting_rate(12_000_000.Hz()),
            126_000_000u32.Hz()
        )
    }
}
