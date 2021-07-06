//! Clocks (CLOCKS)
//!
//!
//!
//! Usage:
//! ```rust
//! let mut p = rp2040_pac::Peripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(p.WATCHDOG);
//! let mut clocks = ClocksManager::new(p.CLOCKS, &mut watchdog);
//! // Enable the xosc
//! let xosc = setup_xosc_blocking(p.XOSC, XOSC_MHZ.Hz()).ok().unwrap();
//!
//!
//! // Configure PLLs
//! //                   REF     FBDIV VCO            POSTDIV
//! // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
//! // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
//! let pll_sys =
//!     setup_pll_blocking(p.PLL_SYS, 12.MHz().into(), PLL_SYS_125MHZ, &mut clocks, &mut p.RESETS).ok().unwrap();
//! let pll_usb =
//!     setup_pll_blocking(p.PLL_USB, 12.MHz().into(), PLL_USB_48MHZ, &mut clocks, &mut p.RESETS).ok().unwrap();
//! clocks.init(&xosc, &pll_sys, &pll_usb);
//! ```
//!
//! See [Chapter 2 Section 15](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use crate::{
    pll::{Locked, PhaseLockedLoop},
    watchdog::Watchdog,
    xosc::{CrystalOscillator, Stable},
};
use core::{convert::TryInto, marker::PhantomData};
use embedded_time::rate::*;
use pac::{clocks, CLOCKS, PLL_SYS, PLL_USB};

#[macro_use]
mod macros;
#[derive(Copy, Clone)]
/// Provides refs to the CLOCKS block.
pub struct ShareableClocks {
    _internal: (),
}

impl ShareableClocks {
    fn new(_clocks: &mut CLOCKS) -> Self {
        ShareableClocks { _internal: () }
    }

    unsafe fn get(&self) -> &clocks::RegisterBlock {
        &*CLOCKS::ptr()
    }
}

const XOSC_MHZ: u32 = 12_000_000_u32;

fn make_div<S: TryInto<Hertz<u64>>, F: TryInto<Hertz<u64>>>(
    src_freq: S,
    freq: F,
) -> Result<u32, ()> {
    let src_freq = *src_freq.try_into().map_err(|_| ())?.integer();
    let freq = *freq.try_into().map_err(|_| ())?.integer();
    let div: u64 = (src_freq << 8).wrapping_div(freq);
    Ok(div as u32)
}
/// Abstraction layer providing Clock Management.
pub struct ClocksManager {
    clocks: CLOCKS,
    shared_clocks: ShareableClocks,
}
impl ClocksManager {
    /// Exchanges CLOCKS block against Self.
    pub fn new(mut clocks_block: CLOCKS, watchdog: &mut Watchdog) -> Self {
        // Start tick in watchdog
        watchdog.enable_tick_generation(XOSC_MHZ as u8);

        // Disable resus that may be enabled from previous software
        clocks_block.clk_sys_resus_ctrl.write_with_zero(|w| w);

        let shared_clocks = ShareableClocks::new(&mut clocks_block);
        ClocksManager {
            clocks: clocks_block,
            shared_clocks,
        }
    }

    /// Initialize the clocks
    pub fn init(
        &self,
        _: &CrystalOscillator<Stable>,
        _: &PhaseLockedLoop<Locked, PLL_SYS>,
        _: &PhaseLockedLoop<Locked, PLL_USB>,
    ) {
        // Configure clocks
        // CLK_REF = XOSC (12MHz) / 1 = 12MHz
        let mut ref_clock = self.reference_clock();
        let div = make_div(12u32.MHz(), 12u32.MHz()).unwrap();
        // If increasing divisor, set divisor before source.
        if div > ref_clock.get_div() {
            ref_clock.set_div(div);
        }
        let clock_token = ref_clock.set_xosc_src();
        nb::block!(ref_clock.await_select(&clock_token)).unwrap();
        ref_clock.set_div(div);

        // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
        let mut sys_clock = self.system_clock();
        let div = make_div(125u32.MHz(), 125u32.MHz()).unwrap();
        // If increasing divisor, set divisor before source.
        if div > sys_clock.get_div() {
            sys_clock.set_div(div);
        }
        sys_clock.set_pll_sys_auxsrc();
        let clock_token = sys_clock.set_self_aux_src();
        nb::block!(sys_clock.await_select(&clock_token)).unwrap();
        sys_clock.set_div(div);

        // CLK USB = PLL USB (48MHz) / 1 = 48MHz
        let mut usb_clock = self.usb_clock();
        let div = make_div(48u32.MHz(), 48u32.MHz()).unwrap();
        // If increasing divisor, set divisor before source.
        if div > usb_clock.get_div() {
            usb_clock.set_div(div);
        }
        usb_clock.disable();
        usb_clock.set_pll_usb_auxsrc();
        usb_clock.enable();
        usb_clock.set_div(div);

        // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
        let mut adc_clock = self.adc_clock();
        let div = make_div(48u32.MHz(), 48u32.MHz()).unwrap();
        // If increasing divisor, set divisor before source.
        if div > adc_clock.get_div() {
            adc_clock.set_div(div);
        }
        adc_clock.disable();
        adc_clock.set_pll_usb_auxsrc();
        adc_clock.enable();
        adc_clock.set_div(div);

        // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
        let mut rtc_clock = self.rtc_clock();
        let div = make_div(48u32.MHz(), 46875u32.Hz()).unwrap();
        // If increasing divisor, set divisor before source.
        if div > rtc_clock.get_div() {
            rtc_clock.set_div(div);
        }
        rtc_clock.disable();
        rtc_clock.set_pll_usb_auxsrc();
        rtc_clock.enable();
        rtc_clock.set_div(div);

        // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
        // Normally choose clk_sys or clk_usb
        let mut peripheral_clock = self.peripheral_clock();
        peripheral_clock.disable();
        peripheral_clock.set_clksys_auxsrc();
        peripheral_clock.enable();
    }

    /// Releases the CLOCKS block
    pub fn free(self) -> CLOCKS {
        self.clocks
    }
}

/// For clocks with a divider
pub trait ClockDivision {
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
    fn await_select(
        &self,
        clock_token: &ChangingClockToken<Self::Clock>,
    ) -> nb::Result<(), ()>;
}

/// Token which can be used to await the glitchless switch
pub struct ChangingClockToken<G: GlitchlessClock> {
    clock_nr: u8,
    clock: PhantomData<G>,
}

/// For clocks that can have XOSC as source.
pub trait XOSCClockSource<G: GlitchlessClock> {
    /// Set XOSC as a source.
    fn set_xosc_src(&mut self) -> ChangingClockToken<G>;
}
/// For clocks that can have ROSC as source.
pub trait ROSCClockSource<G: GlitchlessClock> {
    /// set ROSC as a source.
    fn set_rosc_src(&mut self) -> ChangingClockToken<G>;
}
/// For clocks that also have an aux source
pub trait SelfAuxClockSource<G: GlitchlessClock> {
    /// Set ...
    fn set_self_aux_src(&mut self) -> ChangingClockToken<G>;
}
/// For clocks that can have the Reference Clock as source.
pub trait ClockREFClockSource<G: GlitchlessClock> {
    /// Set Reference Clock as
    fn set_clkref_src(&mut self) -> ChangingClockToken<G>;
}
/// For clocks that can have the System Clock as an auxilliary source.
pub trait ClockSYSClockAuxSource {
    /// Set System Clock as source.
    fn set_clksys_auxsrc(&mut self);
}

/// For clocks that can have the USB Clock as an auxilliary source.
pub trait ClockUSBClockAuxSource {
    /// Set USB Clock as source.
    fn set_clkusb_auxsrc(&mut self);
}

/// For clocks that can have the ADC Clock as an auxilliary source.
pub trait ClockADCClockAuxSource {
    /// Set ADC Clock as source.
    fn set_clkadc_auxsrc(&mut self);
}

/// For clocks that can have the RTC Clock as an auxilliary source.
pub trait ClockRTCClockAuxSource {
    /// Set RTC Clock as source.
    fn set_clkrtc_auxsrc(&mut self);
}

/// For clocks that can have the Reference Clock as an auxilliary source.
pub trait ClockRefClockAuxSource {
    /// Set Reference Clock as source.
    fn set_clkref_auxsrc(&mut self);
}
/// For clocks that can have XOSC as an auxilliary source.
pub trait XOSCClockAuxSource {
    /// Set XOSC as auxilliary source.
    fn set_xosc_auxsrc(&mut self);
}
/// For clocks that can have ROSC as an auxilliary source.
pub trait ROSCClockAuxSource {
    /// Set ROSC as auxilliary source.
    fn set_rosc_auxsrc(&mut self);
}
/// For clocks that can have ROSC_PH as an auxilliary source.
pub trait ROSCPHClockAuxSource {
    /// Set ROSC_PH as auxilliary source.
    fn set_rosc_ph_auxsrc(&mut self);
}
/// For clocks that can have PLL_USB as an auxilliary source.
pub trait PLLUSBClockAuxSource {
    /// Set PLL_USB as auxilliary source.
    fn set_pll_usb_auxsrc(&mut self);
}
/// For clocks that can have PLL_SYS as an auxilliary source.
pub trait PLLSYSClockAuxSource {
    /// Set PLL_SYS as auxilliary source.
    fn set_pll_sys_auxsrc(&mut self);
}
/// For clocks that can have gpin0 as an auxilliary source.
pub trait Gpin0ClockAuxSource {
    /// Set clock to be received from gpin0 (auxilliary)
    fn set_gpin0_auxsrc(&mut self);
}
/// For clocks that can have gpin1 as an auxilliary source.
pub trait Gpin1ClockAuxSource {
    /// Set clock to be received from gpin1
    fn set_gpin1_auxsrc(&mut self);
}

/// For clocks having a generator.
pub trait ClockGenerator {
    /// Enables the clock.
    fn enable(&mut self);

    /// Disables the clock.
    fn disable(&mut self);

    /// Kills the clock.
    fn kill(&mut self);
}

clock! {
    /// GPIO Output 0 Clock
    (GpioOutput0Clock, clk_gpout0, auxsrc=[pll_sys, gpin0, gpin1, pll_usb, rosc, xosc, clocksys, clockusb, clockadc, clockrtc, clockref])
}
clock! {
    /// GPIO Output 1 Clock
    (GpioOutput1Clock, clk_gpout1, auxsrc=[gpin0, gpin1, pll_usb, rosc, xosc, clocksys, clockusb, clockadc, clockrtc, clockref])
}
clock! {
    /// GPIO Output 2 Clock
    (GpioOutput2Clock, clk_gpout2, auxsrc=[gpin0, gpin1, pll_usb, rosc_ph, xosc, clocksys, clockusb, clockadc, clockrtc, clockref])
}
clock! {
    /// GPIO Output 3 Clock
    (GpioOutput3Clock, clk_gpout3, auxsrc=[gpin0, gpin1, pll_usb, rosc_ph, xosc, clocksys, clockusb, clockadc, clockrtc, clockref])
}
clock! {
    /// Reference Clock
    (ReferenceClock, clk_ref, src=[rosc, selfaux, xosc], auxsrc=[pll_usb, gpin0, gpin1])
}
clock! {
    /// System Clock
    (SystemClock, clk_sys, src=[clockref, selfaux], auxsrc=[pll_sys, pll_usb, rosc, xosc, gpin0, gpin1])
}
clock! {
    /// Peripheral Clock
    (PeripheralClock, clk_peri, auxsrc=[clocksys, pll_sys, pll_usb, rosc_ph, xosc, gpin0, gpin1], nodiv)
}
clock! {
    /// USB Clock
    (UsbClock, clk_usb, auxsrc=[pll_usb, pll_sys, rosc_ph, xosc, gpin0, gpin1])
}
clock! {
    /// Adc Clock
    (AdcClock, clk_adc, auxsrc=[pll_usb, pll_sys, rosc_ph, xosc, gpin0, gpin1])
}
clock! {
    /// RTC Clock
    (RtcClock, clk_rtc, auxsrc=[pll_usb, pll_sys, rosc_ph, xosc, gpin0, gpin1])
}

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
