//! Phase-Locked Loops (PLL)
// See [Chapter 2 Section 18](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use core::{
    convert::{
        Infallible,
        TryFrom
    },
    marker::PhantomData,
    ops::{
        RangeInclusive,
        Range,
        Deref
    }
};

use embedded_time::{
    fixed_point::FixedPoint,
    rate::{
        Hertz,
        Generic
    }
};

use nb::Error::WouldBlock;

/// State of the PLL
pub trait State {}


/// PLL is disabled but is configured.
pub struct Disabled {
    refdiv: u8,
    vco_freq: Hertz,
    post_div1: u8,
    post_div2: u8
}

/// PLL is locked : it delivers a steady frequency.
pub struct Locked;

/// PLL is locking into its designated frequency.
pub struct Locking {
    post_div1: u8,
    post_div2: u8
}

impl State for Disabled {}
impl State for Locked {}
impl State for Locking {}


/// Trait to handle both underlying devices from the PAC (PLL_SYS & PLL_USB)
pub trait PhaseLockedLoopDevice: Deref<Target = rp2040_pac::pll_sys::RegisterBlock> {}

impl PhaseLockedLoopDevice for rp2040_pac::PLL_SYS {}
impl PhaseLockedLoopDevice for rp2040_pac::PLL_USB {}

/// A PLL.
pub struct PhaseLockedLoop<S: State, D: PhaseLockedLoopDevice> {
    device: D,
    state: S
}

impl<S: State, D: PhaseLockedLoopDevice> PhaseLockedLoop<S, D> {
    fn transition<To: State>(self, state: To) -> PhaseLockedLoop<To, D> {
        PhaseLockedLoop {
            device: self.device,
            state: state
        }
    }

    /// Releases the underlying device.
    pub fn free(self) -> D{
        self.device
    }
}


/// Error type for the PLL module.
/// See Chapter 2, Section 18 §2 for details on constraints triggering these errors.
pub enum Error {

    /// Proposed VCO frequency is out of range.
    VCOFreqOutOfRange,

    /// Feedback Divider value is out of range.
    FBDIVOutOfRange,

    /// Post Divider value is out of range.
    PostDivOutOfRage,

    /// Reference Frequency is out of range.
    RefFreqOutOfRange,

    /// Bad argument : overflows, bad conversion, ...
    BadArgument
}

impl<D: PhaseLockedLoopDevice> PhaseLockedLoop<Disabled, D> {


    /// Instantiates and configures a new Phase-Locked-Loop device.
    pub fn new(dev: D, refdiv: u8, vco_freq: Generic<u32>, post_div1: u8, post_div2: u8) -> Result<PhaseLockedLoop<Disabled, D>, Error> {

        const VCO_FREQ_RANGE: RangeInclusive<Hertz<u32>> = Hertz(400_000_000)..=Hertz(1600_000_000);
        const POSTDIV_RANGE: Range<u8> = 1..7;

        let vco_freq = Hertz::<u32>::try_from(vco_freq).map_err(|_| Error::BadArgument)?;

        if !VCO_FREQ_RANGE.contains(&vco_freq) {
            return Err(Error::VCOFreqOutOfRange)
        }

        if !POSTDIV_RANGE.contains(&post_div1) || !POSTDIV_RANGE.contains(&post_div2) {
            return Err(Error::PostDivOutOfRage)
        }

        Ok(PhaseLockedLoop {
            state: Disabled {
                refdiv, vco_freq, post_div1, post_div2
            },
            device: dev,
        })
    }

    /// Configures and starts the PLL : it switches to Locking state.
    pub fn initialize(self, xosc_frequency: Generic<u32>) -> Result<PhaseLockedLoop<Locking, D>, Error>{

        const FBDIV_RANGE: Range<u16> = 16..320;

        let ref_freq_range: Range<Hertz<u32>> = Hertz(5_000_000)..self.state.vco_freq.div(16);

        // Turn off PLL in case it is already running
        self.device.pwr.reset();
        self.device.fbdiv_int.reset();

        let refdiv = self.state.refdiv;

        let ref_freq_hz = Hertz::<u32>::try_from(xosc_frequency).
            map_err(|_| Error::BadArgument)?.
            checked_div(&(refdiv as u32)).
            ok_or(Error::BadArgument)?;

        if !ref_freq_range.contains(&ref_freq_hz) {
            return Err(Error::RefFreqOutOfRange)
        }

        self.device.cs.write(|w| unsafe {
            w.refdiv().bits(refdiv as u8);
            w
        });

        let fbdiv = *self.state.vco_freq.checked_div(ref_freq_hz.integer()).
            ok_or(Error::BadArgument)?.integer() as u16;

        if !FBDIV_RANGE.contains(&fbdiv) {
            return Err(Error::FBDIVOutOfRange)
        }

        self.device.fbdiv_int.write(|w| unsafe {
            w.fbdiv_int().bits(fbdiv);
            w
        });

        self.device.cs.write(|w| unsafe {
            w.refdiv().bits(refdiv);
            w
        });

        // Turn on self.device
        self.device.pwr.write(|w| unsafe {
            //w.pd().clear_bit();
            //w.vcopd().clear_bit();
            w.bits(0);
            w
        });

        let post_div1 = self.state.post_div1;
        let post_div2 = self.state.post_div2;

        Ok(self.transition(Locking {
            post_div1, post_div2
        }))
    }
}

/// A token that's given when the PLL is properly locked, so we can safely transition to the next state.
pub struct LockedPLLToken<D> {
    _private: PhantomData<D>
}

impl<D: PhaseLockedLoopDevice> PhaseLockedLoop<Locking, D> {

    /// Awaits locking of the PLL.
    pub fn await_lock(&self) -> nb::Result<LockedPLLToken<D>, Infallible> {

        if self.device.cs.read().lock().bit_is_clear() {
            return Err(WouldBlock);
        }

        Ok(LockedPLLToken {
            _private: PhantomData
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
        self.device.pwr.write(|w| unsafe {
            //w.postdivpd().clear_bit();
            w.bits(0);
            w
        });

        self.transition(Locked)
    }

}
