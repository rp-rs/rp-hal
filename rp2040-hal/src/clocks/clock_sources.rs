//! Available clocks

use super::*;
use crate::{
    gpin,
    gpio::{PullNone, PullType},
    rosc::{Enabled, RingOscillator},
};

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

// GPIN0
pub(crate) type GpIn0<M = PullNone> = gpin::GpIn0<M>;
impl<M: PullType> ClockSource for GpIn0<M> {
    fn get_freq(&self) -> HertzU32 {
        self.frequency()
    }
}

// GPIN1
pub(crate) type GpIn1<M = PullNone> = gpin::GpIn1<M>;
impl<M: PullType> ClockSource for GpIn1<M> {
    fn get_freq(&self) -> HertzU32 {
        self.frequency()
    }
}
