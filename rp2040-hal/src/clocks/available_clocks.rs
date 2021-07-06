//! Available clocks

use super::{ClocksManager, ShareableClocks};
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
use core::{convert::TryInto, marker::PhantomData};
use embedded_time::rate::*;
use pac::{PLL_SYS, PLL_USB};

fn make_div<S: TryInto<Hertz<u64>>, F: TryInto<Hertz<u64>>>(
    src_freq: S,
    freq: F,
) -> Result<u32, ()> {
    let src_freq = *src_freq.try_into().map_err(|_| ())?.integer();
    let freq = *freq.try_into().map_err(|_| ())?.integer();
    let div: u64 = (src_freq << 8).wrapping_div(freq);
    Ok(div as u32)
}

/// For clocks with a divider
trait ClockDivision {
    /// Set integer divider value.
    fn set_div(&mut self, div: u32);
    /// Get integer diveder value.
    fn get_div(&self) -> u32;
}

/// Clock with glitchless source
pub trait GlitchlessClock {
    /// Self type to hand to ChangingClockToken
    type Clock: GlitchlessClock;

    /// Await switching clock sources without glitches. Needs a token that is returned when setting
    fn await_select(&self, clock_token: &ChangingClockToken<Self::Clock>) -> nb::Result<(), ()>;
}

/// Token which can be used to await the glitchless switch
pub struct ChangingClockToken<G: GlitchlessClock> {
    clock_nr: u8,
    clock: PhantomData<G>,
}

/// For clocks that can be disabled
pub trait StoppableClock {
    /// Enables the clock.
    fn enable(&mut self);

    /// Disables the clock.
    fn disable(&mut self);

    /// Kills the clock.
    fn kill(&mut self);
}

/// Trait for things that can be used as clock source
pub trait ClockSource: Sealed {
    /// Get the operating frequency for this source
    ///
    /// Used to determine the divisor
    fn get_freq(&self) -> Hertz;
}

type PllSys = PhaseLockedLoop<Locked, PLL_SYS>;
impl Sealed for PllSys {}
impl ClockSource for PllSys {
    fn get_freq(&self) -> Hertz {
        self.operating_frequency()
    }
}

type PllUsb = PhaseLockedLoop<Locked, PLL_USB>;
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

type Xosc = CrystalOscillator<Stable>;
impl Sealed for Xosc {}
impl ClockSource for Xosc {
    fn get_freq(&self) -> Hertz {
        self.operating_frequency()
    }
}

type Rosc = RingOscillator<Enabled>;
impl Sealed for Rosc {}
// We are assuming the second output is never phase shifted (see 2.17.4)
impl ClockSource for RingOscillator<Enabled> {
    fn get_freq(&self) -> Hertz {
        self.operating_frequency()
    }
}

// GPIN0
type GPin0 = Pin<Gpio20, FunctionClock>;
impl ClockSource for GPin0 {
    fn get_freq(&self) -> Hertz {
        todo!()
    }
}

// GPIN1
type GPin1 = Pin<Gpio22, FunctionClock>;
impl ClockSource for Pin<Gpio22, FunctionClock> {
    fn get_freq(&self) -> Hertz {
        todo!()
    }
}

/// Trait to contrain which ClockSource is valid for which Clock
pub trait ValidSrc<Clock>: Sealed {
    /// Which register values are acceptable
    type Variant;

    /// Is this a ClockSource for src or aux?
    fn is_aux(&self) -> bool;
    /// Get register value for this ClockSource
    fn variant(&self) -> Self::Variant;
}

clock!(
    /// GPIO Output 0 Clock
    struct GpioOutput0Clock {
        reg: clk_gpout0,
        auxsrc: {PllSys:CLKSRC_PLL_SYS, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC, Xosc: XOSC_CLKSRC, SystemClock: CLK_SYS, UsbClock: CLK_USB, AdcClock: CLK_ADC, RtcClock: CLK_RTC, ReferenceClock:CLK_REF}
    }
);
clock!(
    /// GPIO Output 1 Clock
    struct GpioOutput1Clock {
        reg: clk_gpout1,
        auxsrc: {PllSys:CLKSRC_PLL_SYS, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC, Xosc: XOSC_CLKSRC, SystemClock: CLK_SYS, UsbClock: CLK_USB, AdcClock: CLK_ADC, RtcClock: CLK_RTC, ReferenceClock:CLK_REF}
    }
);
clock!(
    /// GPIO Output 2 Clock
    struct GpioOutput2Clock {
        reg: clk_gpout2,
        auxsrc: {PllSys:CLKSRC_PLL_SYS, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC, SystemClock: CLK_SYS, UsbClock: CLK_USB, AdcClock: CLK_ADC, RtcClock: CLK_RTC, ReferenceClock:CLK_REF}
    }
);
clock!(
    /// GPIO Output 3 Clock
    struct GpioOutput3Clock {
        reg: clk_gpout3,
        auxsrc: {PllSys:CLKSRC_PLL_SYS, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC, SystemClock: CLK_SYS, UsbClock: CLK_USB, AdcClock: CLK_ADC, RtcClock: CLK_RTC, ReferenceClock:CLK_REF}
    }
);
clock!(
    /// Reference Clock
    struct ReferenceClock {
        reg: clk_ref,
        src: {Rosc: ROSC_CLKSRC_PH, Xosc:XOSC_CLKSRC},
        auxsrc: {PllUsb:CLKSRC_PLL_USB, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
);
clock!(
    /// System Clock
    struct SystemClock {
        reg: clk_sys,
        src: {ReferenceClock: CLK_REF},
        auxsrc: {PllSys: CLKSRC_PLL_SYS, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
);
clock!(
    /// Peripheral Clock
    struct PeripheralClock {
        reg: clk_peri,
        auxsrc: {SystemClock: CLK_SYS, PllSys: CLKSRC_PLL_SYS, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1 },
        div: false
    }
);
clock!(
    /// USB Clock
    struct UsbClock {
        reg: clk_usb,
        auxsrc: {PllUsb:CLKSRC_PLL_USB,PllSys: CLKSRC_PLL_SYS,  Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
);
clock!(
    /// Adc Clock
    struct AdcClock {
        reg: clk_adc,
        auxsrc: {PllUsb:CLKSRC_PLL_USB,PllSys: CLKSRC_PLL_SYS,  Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
);
clock!(
    /// RTC Clock
    struct RtcClock {
        reg: clk_rtc,
        auxsrc: {PllUsb:CLKSRC_PLL_USB,PllSys: CLKSRC_PLL_SYS,  Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
);

impl SystemClock {
    fn get_default_clock_source(&self) -> pac::clocks::clk_sys_ctrl::SRC_A {
        pac::clocks::clk_sys_ctrl::SRC_A::CLK_REF
    }

    fn get_aux_source(&self) -> pac::clocks::clk_sys_ctrl::SRC_A {
        pac::clocks::clk_sys_ctrl::SRC_A::CLKSRC_CLK_SYS_AUX
    }
}

impl ReferenceClock {
    fn get_default_clock_source(&self) -> pac::clocks::clk_ref_ctrl::SRC_A {
        pac::clocks::clk_ref_ctrl::SRC_A::ROSC_CLKSRC_PH
    }

    fn get_aux_source(&self) -> pac::clocks::clk_ref_ctrl::SRC_A {
        pac::clocks::clk_ref_ctrl::SRC_A::CLKSRC_CLK_REF_AUX
    }
}
