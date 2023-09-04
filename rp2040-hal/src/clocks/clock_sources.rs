//! Available clocks

use super::*;
use crate::{
    gpio::{
        bank0::{Gpio20, Gpio22},
        FunctionClock, Pin, PullNone, PullType,
    },
    pll::{Locked, PhaseLockedLoop},
    rosc::{Enabled, RingOscillator},
    typelevel::Sealed,
    xosc::{CrystalOscillator, Stable},
};
use pac::{PLL_SYS, PLL_USB};

pub(crate) type PllSys = PhaseLockedLoop<Locked, PLL_SYS>;
impl Sealed for PllSys {}
impl ClockSource for PllSys {
    fn get_freq(&self) -> HertzU32 {
        self.operating_frequency()
    }
}

pub(crate) type PllUsb = PhaseLockedLoop<Locked, PLL_USB>;
impl Sealed for PllUsb {}
impl ClockSource for PllUsb {
    fn get_freq(&self) -> HertzU32 {
        self.operating_frequency()
    }
}

impl ClockSource for UsbClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

impl ClockSource for AdcClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

impl ClockSource for RtcClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

impl ClockSource for SystemClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

impl ClockSource for ReferenceClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

pub(crate) type Xosc = CrystalOscillator<Stable>;
impl Sealed for Xosc {}
impl ClockSource for Xosc {
    fn get_freq(&self) -> HertzU32 {
        self.operating_frequency()
    }
}

pub(crate) type Rosc = RingOscillator<Enabled>;
impl Sealed for Rosc {}
// We are assuming the second output is never phase shifted (see 2.17.4)
impl ClockSource for RingOscillator<Enabled> {
    fn get_freq(&self) -> HertzU32 {
        self.operating_frequency()
    }
}

/// Gpio20 in clock function associated with a frequency.
pub struct GPin0<M: PullType = PullNone>(Pin<Gpio20, FunctionClock, M>, HertzU32);
impl<M: PullType> GPin0<M> {
    /// Assemble Gpio20 and a frequency into a clock source.
    pub fn new(p: Pin<Gpio20, FunctionClock, M>, freq: HertzU32) -> Self {
        GPin0(p, freq)
    }
}
impl<M: PullType> Sealed for GPin0<M> {}
impl<M: PullType> ClockSource for GPin0<M> {
    fn get_freq(&self) -> HertzU32 {
        self.1
    }
}

/// Gpio22 in clock function associated with a frequency.
pub struct GPin1<M: PullType = PullNone>(Pin<Gpio22, FunctionClock, M>, HertzU32);
impl<M: PullType> GPin1<M> {
    /// Assemble Gpio22 and a frequency into a clock source.
    pub fn new(p: Pin<Gpio22, FunctionClock, M>, freq: HertzU32) -> Self {
        GPin1(p, freq)
    }
}
impl<M: PullType> Sealed for GPin1<M> {}
impl<M: PullType> ClockSource for GPin1<M> {
    fn get_freq(&self) -> HertzU32 {
        self.1
    }
}
