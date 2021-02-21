//! Clock tree configuration
//!
//! # Example
/// ```rust no_run
/// use embedded_time::rate::Extensions as _;
///
/// use rp2040_pac::Peripherals;
/// use rp2040_hal::clocks::{XOsc, SplitClocks};
/// use rp2040_hal::clocks::pll::Pll;
///
/// let p = Peripherals::take().unwrap();
///
/// // Setup the clock sources
/// let xosc = XOsc::new_stable_blocking(p.XOSC, 12.MHz().into()).unwrap();
/// let pll_sys = Pll::new_stable_blocking(p.PLL_SYS, &xosc, 125.MHz().into()).unwrap();
/// let pll_usb = Pll::new_stable_blocking(p.PLL_USB, &xosc, 48.MHz().into()).unwrap();
///
/// // Derive the clocks for the different domains
/// let clocks = SplitClocks::split(p.CLOCKS);
///
/// let clk_peri = clocks.peri;
/// // clk_peri.switch_to(&pll_sys);
///
/// // ToDo: Use UART
/// ```
pub mod generators;
pub mod pll;
mod xosc;

pub use generators::SplitClocks;
pub use pll::Pll;
pub use xosc::XOsc;

use core::convert::TryFrom;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate;
use embedded_time::rate::*;

pub type Rate = rate::Generic<u32>;

/// Trait to make it easier to express a simple rate
pub trait ClkRate:
    embedded_time::rate::Rate + FixedPoint<T = u32> + Copy + PartialOrd<Megahertz> + From<Hertz>
{
    /// Return the rate converted to Hz if possible
    fn as_hz(&self) -> Option<Hertz>;
}

impl ClkRate for Megahertz {
    fn as_hz(&self) -> Option<Hertz> {
        Hertz::try_from(*self).ok()
    }
}

impl ClkRate for Kilohertz {
    fn as_hz(&self) -> Option<Hertz> {
        Hertz::try_from(*self).ok()
    }
}

impl ClkRate for Hertz {
    fn as_hz(&self) -> Option<Hertz> {
        Some(*self)
    }
}

/// Marker Trait for Clock sourcing devices
pub trait ClkDevice {}

/// Trait for wrappers around clock sources
pub trait ClkSource {
    type DeviceName;

    /// Returns the clock rate of the wrapped device.
    fn rate(&self) -> rate::Generic<u32>;
}
