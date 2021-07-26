//! Phase-Locked Loops (PLL)
// See [Chapter 2 Section 18](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use core::{
    convert::{Infallible, TryFrom, TryInto},
    marker::PhantomData,
    ops::{Deref, Range, RangeInclusive},
};

use embedded_time::{
    fixed_point::FixedPoint,
    rate::{Generic, Hertz, Rate},
};

use nb::Error::WouldBlock;
use pac::RESETS;

use crate::{clocks::ClocksManager, resets::SubsystemReset};

/// State of the PLL
pub trait State {}

/// PLL is disabled.
pub struct Disabled {
    refdiv: u8,
    fbdiv: u16,
    post_div1: u8,
    post_div2: u8,
    frequency: Hertz,
}

/// PLL is configured, started and locking into its designated frequency.
pub struct Locking {
    post_div1: u8,
    post_div2: u8,
    frequency: Hertz,
}

/// PLL is locked : it delivers a steady frequency.
pub struct Locked {
    frequency: Hertz,
}

impl State for Disabled {}
impl State for Locked {}
impl State for Locking {}

/// Trait to handle both underlying devices from the PAC (PLL_SYS & PLL_USB)
pub trait PhaseLockedLoopDevice:
    Deref<Target = rp2040_pac::pll_sys::RegisterBlock> + SubsystemReset
{
}

impl PhaseLockedLoopDevice for rp2040_pac::PLL_SYS {}
impl PhaseLockedLoopDevice for rp2040_pac::PLL_USB {}

/// A PLL.
pub struct PhaseLockedLoop<S: State, D: PhaseLockedLoopDevice> {
    device: D,
    state: S,
}

impl<S: State, D: PhaseLockedLoopDevice> PhaseLockedLoop<S, D> {
    fn transition<To: State>(self, state: To) -> PhaseLockedLoop<To, D> {
        PhaseLockedLoop {
            device: self.device,
            state,
        }
    }

    /// Releases the underlying device.
    pub fn free(self) -> D {
        self.device
    }
}

/// Error type for the PLL module.
/// See Chapter 2, Section 18 §2 for details on constraints triggering these errors.
pub enum Error {
    /// Proposed VCO frequency is out of range.
    VcoFreqOutOfRange,

    /// Feedback Divider value is out of range.
    FeedbackDivOutOfRange,

    /// Post Divider value is out of range.
    PostDivOutOfRage,

    /// Reference Frequency is out of range.
    RefFreqOutOfRange,

    /// Bad argument : overflows, bad conversion, ...
    BadArgument,
}

/// Parameters for a PLL.
pub struct PLLConfig<R: Rate> {
    /// Voltage Controlled Oscillator frequency.
    pub vco_freq: R,

    /// Reference divider
    pub refdiv: u8,

    /// Post Divider 1
    pub post_div1: u8,

    /// Post Divider 2
    pub post_div2: u8,
}

/// Common configs for the two PLLs. Both assume the XOSC is cadenced at 12MHz !
/// See Chapter 2, Section 18, §2
pub mod common_configs {
    use super::PLLConfig;
    use embedded_time::rate::Megahertz;

    /// Default, nominal configuration for PLL_SYS
    pub const PLL_SYS_125MHZ: PLLConfig<Megahertz> = PLLConfig {
        vco_freq: Megahertz(1500),
        refdiv: 1,
        post_div1: 6,
        post_div2: 2,
    };

    /// Default, nominal configuration for PLL_USB.
    pub const PLL_USB_48MHZ: PLLConfig<Megahertz> = PLLConfig {
        vco_freq: Megahertz(480),
        refdiv: 1,
        post_div1: 5,
        post_div2: 2,
    };
}

impl<D: PhaseLockedLoopDevice> PhaseLockedLoop<Disabled, D> {
    /// Instantiates a new Phase-Locked-Loop device.
    pub fn new<R: Rate>(
        dev: D,
        xosc_frequency: Generic<u32>,
        config: PLLConfig<R>,
    ) -> Result<PhaseLockedLoop<Disabled, D>, Error>
    where
        R: Into<Hertz<u64>>,
    {
        const VCO_FREQ_RANGE: RangeInclusive<Hertz<u32>> =
            Hertz(400_000_000)..=Hertz(1_600_000_000);
        const POSTDIV_RANGE: Range<u8> = 1..7;
        const FBDIV_RANGE: Range<u16> = 16..320;

        //First we convert our rate to Hertz<u64> as all other rates can be converted to that.
        let vco_freq: Hertz<u64> = config.vco_freq.into();

        //Then we try to downscale to u32.
        let vco_freq: Hertz<u32> = vco_freq.try_into().map_err(|_| Error::BadArgument)?;

        if !VCO_FREQ_RANGE.contains(&vco_freq) {
            return Err(Error::VcoFreqOutOfRange);
        }

        if !POSTDIV_RANGE.contains(&config.post_div1) || !POSTDIV_RANGE.contains(&config.post_div2)
        {
            return Err(Error::PostDivOutOfRage);
        }

        let ref_freq_range: Range<Hertz<u32>> = Hertz(5_000_000)..vco_freq.div(16);

        let ref_freq_hz = Hertz::<u32>::try_from(xosc_frequency)
            .map_err(|_| Error::BadArgument)?
            .checked_div(&(config.refdiv as u32))
            .ok_or(Error::BadArgument)?;

        if !ref_freq_range.contains(&ref_freq_hz) {
            return Err(Error::RefFreqOutOfRange);
        }

        let fbdiv = vco_freq
            .checked_div(&ref_freq_hz.integer())
            .ok_or(Error::BadArgument)?;

        let fbdiv: u16 = (fbdiv.integer())
            .try_into()
            .map_err(|_| Error::BadArgument)?;

        if !FBDIV_RANGE.contains(&fbdiv) {
            return Err(Error::FeedbackDivOutOfRange);
        }

        let refdiv = config.refdiv;
        let post_div1 = config.post_div1;
        let post_div2 = config.post_div2;
        let frequency: Hertz =
            (ref_freq_hz / refdiv as u32) * fbdiv as u32 / (post_div1 as u32 * post_div2 as u32);

        Ok(PhaseLockedLoop {
            state: Disabled {
                refdiv,
                fbdiv,
                post_div1,
                post_div2,
                frequency,
            },
            device: dev,
        })
    }

    /// Configures and starts the PLL : it switches to Locking state.
    pub fn initialize(self, resets: &mut rp2040_pac::RESETS) -> PhaseLockedLoop<Locking, D> {
        self.device.reset_bring_up(resets);

        // Turn off PLL in case it is already running
        self.device.pwr.reset();
        self.device.fbdiv_int.reset();

        self.device.cs.write(|w| unsafe {
            w.refdiv().bits(self.state.refdiv);
            w
        });

        self.device.fbdiv_int.write(|w| unsafe {
            w.fbdiv_int().bits(self.state.fbdiv);
            w
        });

        // Turn on PLL
        self.device.pwr.modify(|_, w| {
            w.pd().clear_bit();
            w.vcopd().clear_bit();
            w
        });

        let post_div1 = self.state.post_div1;
        let post_div2 = self.state.post_div2;
        let frequency = self.state.frequency;

        self.transition(Locking {
            post_div1,
            post_div2,
            frequency,
        })
    }
}

/// A token that's given when the PLL is properly locked, so we can safely transition to the next state.
pub struct LockedPLLToken<D> {
    _private: PhantomData<D>,
}

impl<D: PhaseLockedLoopDevice> PhaseLockedLoop<Locking, D> {
    /// Awaits locking of the PLL.
    pub fn await_lock(&self) -> nb::Result<LockedPLLToken<D>, Infallible> {
        if self.device.cs.read().lock().bit_is_clear() {
            return Err(WouldBlock);
        }

        Ok(LockedPLLToken {
            _private: PhantomData,
        })
    }

    /// Exchanges a token for a Locked PLL.
    pub fn get_locked(self, _token: LockedPLLToken<D>) -> PhaseLockedLoop<Locked, D> {
        // Set up post dividers
        self.device.prim.write(|w| unsafe {
            w.postdiv1().bits(self.state.post_div1);
            w.postdiv2().bits(self.state.post_div2);
            w
        });

        // Turn on post divider
        self.device.pwr.modify(|_, w| {
            w.postdivpd().clear_bit();
            w
        });

        let frequency = self.state.frequency;

        self.transition(Locked { frequency })
    }
}

impl<D: PhaseLockedLoopDevice> PhaseLockedLoop<Locked, D> {
    /// Get the operating frequency for the PLL
    pub fn operating_frequency(&self) -> Hertz {
        self.state.frequency
    }
}

/// Blocking helper method to setup the PLL without going through all the steps.
pub fn setup_pll_blocking<D: PhaseLockedLoopDevice, R: Rate>(
    dev: D,
    xosc_frequency: Generic<u32>,
    config: PLLConfig<R>,
    clocks: &mut ClocksManager,
    resets: &mut RESETS,
) -> Result<PhaseLockedLoop<Locked, D>, Error>
where
    R: Into<Hertz<u64>>,
{
    // Before we touch PLLs, switch sys and ref cleanly away from their aux sources.
    nb::block!(clocks.system_clock.reset_source_await()).unwrap();

    nb::block!(clocks.reference_clock.reset_source_await()).unwrap();

    let initialized_pll = PhaseLockedLoop::new(dev, xosc_frequency, config)?.initialize(resets);

    let locked_pll_token = nb::block!(initialized_pll.await_lock()).unwrap();

    Ok(initialized_pll.get_locked(locked_pll_token))
}
