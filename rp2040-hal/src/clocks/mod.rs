//! Clock tree configuration

pub mod pll;
mod xosc;

pub use pll::Pll;
pub use xosc::XOsc;

use core::convert::TryFrom;
use embedded_time::fixed_point::FixedPoint;
use embedded_time::rate::*;

/// Trait to make it easier to express a simple rate
pub trait ClkRate: Rate + FixedPoint<T = u32> + Copy + PartialOrd<Megahertz> + From<Hertz> {
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
pub trait ClkSource<Dev: ClkDevice, R: ClkRate> {
    /// Returns the clock rate of the wrapped device.
    fn rate(&self) -> R;
}
