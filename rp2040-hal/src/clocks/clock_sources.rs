//! Available clocks

use super::*;
use crate::{
    gpio::{
        bank0::{Gpio20, Gpio22},
        FunctionClock, Pin,
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
    fn get_freq(&self) -> Hertz {
        self.operating_frequency()
    }
}

pub(crate) type PllUsb = PhaseLockedLoop<Locked, PLL_USB>;
impl Sealed for PllUsb {}
impl ClockSource for PllUsb {
    fn get_freq(&self) -> Hertz {
        self.operating_frequency()
    }
}

impl ClockSource for UsbClock {
    fn get_freq(&self) -> Hertz {
        self.frequency
    }
}

impl ClockSource for AdcClock {
    fn get_freq(&self) -> Hertz {
        self.frequency
    }
}

impl ClockSource for RtcClock {
    fn get_freq(&self) -> Hertz {
        self.frequency
    }
}

impl ClockSource for SystemClock {
    fn get_freq(&self) -> Hertz {
        self.frequency
    }
}

impl ClockSource for ReferenceClock {
    fn get_freq(&self) -> Hertz {
        self.frequency
    }
}

pub(crate) type Xosc = CrystalOscillator<Stable>;
impl Sealed for Xosc {}
impl ClockSource for Xosc {
    fn get_freq(&self) -> Hertz {
        self.operating_frequency()
    }
}

pub(crate) type Rosc = RingOscillator<Enabled>;
impl Sealed for Rosc {}
// We are assuming the second output is never phase shifted (see 2.17.4)
impl ClockSource for RingOscillator<Enabled> {
    fn get_freq(&self) -> Hertz {
        self.operating_frequency()
    }
}

// GPIN0
pub(crate) type GPin0 = Pin<Gpio20, FunctionClock>;
impl ClockSource for GPin0 {
    fn get_freq(&self) -> Hertz {
        todo!()
    }
}

// GPIN1
pub(crate) type GPin1 = Pin<Gpio22, FunctionClock>;
impl ClockSource for Pin<Gpio22, FunctionClock> {
    fn get_freq(&self) -> Hertz {
        todo!()
    }
}
