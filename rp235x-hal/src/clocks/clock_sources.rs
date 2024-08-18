//! Available clocks

use super::*;
use crate::{
    gpio::{
        bank0::{Gpio20, Gpio22},
        FunctionClock, Pin, PullNone, PullType,
    },
    lposc::LowPowerOscillator,
    pll::{Locked, PhaseLockedLoop},
    rosc::{Enabled as RingOscillatorEnabled, RingOscillator},
    typelevel::Sealed,
    xosc::{CrystalOscillator, Stable},
};
use pac::{PLL_SYS, PLL_USB};

/// System PLL.
pub(crate) type PllSys = PhaseLockedLoop<Locked, PLL_SYS>;
impl Sealed for PllSys {}
impl ClockSource for PllSys {
    fn get_freq(&self) -> HertzU32 {
        self.operating_frequency()
    }
}

/// USB PLL.
pub(crate) type PllUsb = PhaseLockedLoop<Locked, PLL_USB>;
impl Sealed for PllUsb {}
impl ClockSource for PllUsb {
    fn get_freq(&self) -> HertzU32 {
        self.operating_frequency()
    }
}

// The USB Clock Generator is a clock source
impl ClockSource for UsbClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

// The ADC Clock Generator is a clock source
impl ClockSource for AdcClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

// The HSTX Clock Generator is a clock source
impl ClockSource for HstxClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

// The System Clock Generator is a clock source
impl ClockSource for SystemClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

// The Reference Clock Generator is a clock source
impl ClockSource for ReferenceClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

// The Peripheral Clock Generator is a clock source
impl ClockSource for PeripheralClock {
    fn get_freq(&self) -> HertzU32 {
        self.frequency
    }
}

// The Low Power Oscillator is a clock source
pub(crate) type LpOsc = LowPowerOscillator;
impl ClockSource for LpOsc {
    fn get_freq(&self) -> HertzU32 {
        32768.Hz()
    }
}

// The Crystal Oscillator
pub(crate) type Xosc = CrystalOscillator<Stable>;
impl Sealed for Xosc {}
impl ClockSource for Xosc {
    fn get_freq(&self) -> HertzU32 {
        self.operating_frequency()
    }
}

// The Ring Oscillator
pub(crate) type Rosc = RingOscillator<RingOscillatorEnabled>;
impl Sealed for Rosc {}
// We are assuming the second output is never phase shifted (see 2.17.4)
impl ClockSource for RingOscillator<RingOscillatorEnabled> {
    fn get_freq(&self) -> HertzU32 {
        self.operating_frequency()
    }
}

// GPIN0
pub(crate) type GPin0<M = PullNone> = Pin<Gpio20, FunctionClock, M>;
impl<M: PullType> ClockSource for GPin0<M> {
    fn get_freq(&self) -> HertzU32 {
        todo!()
    }
}

// GPIN1
pub(crate) type GPin1<M = PullNone> = Pin<Gpio22, FunctionClock, M>;
impl<M: PullType> ClockSource for Pin<Gpio22, FunctionClock, M> {
    fn get_freq(&self) -> HertzU32 {
        todo!()
    }
}
