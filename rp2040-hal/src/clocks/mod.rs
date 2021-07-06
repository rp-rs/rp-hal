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
use core::convert::TryInto;
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
        let mut ref_clock = self.ref_clock();
        let div = make_div(12u32.MHz(), 12u32.MHz()).unwrap();
        // If increasing divisor, set divisor before source.
        if div > ref_clock.get_div() {
            ref_clock.set_div(div);
        }
        ref_clock.set_xosc_src();
        ref_clock.await_select(2);
        ref_clock.set_div(div);

        // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
        let mut sys_clock = self.sys_clock();
        let div = make_div(125u32.MHz(), 125u32.MHz()).unwrap();
        // If increasing divisor, set divisor before source.
        if div > sys_clock.get_div() {
            sys_clock.set_div(div);
        }
        sys_clock.set_pll_sys_auxsrc();
        sys_clock.set_self_aux_src();
        sys_clock.await_select(1);
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

    /// Getter for the GPIO Output 0 Clock.
    pub fn gpio_output0_clock(&self) -> GpioOutput0Clock {
        GpioOutput0Clock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the GPIO Output 1 Clock.
    pub fn gpio_output1_clock(&self) -> GpioOutput1Clock {
        GpioOutput1Clock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the GPIO Output 2 Clock.
    pub fn gpio_output2_clock(&self) -> GpioOutput2Clock {
        GpioOutput2Clock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the GPIO Output 3 Clock.
    pub fn gpio_output3_clock(&self) -> GpioOutput3Clock {
        GpioOutput3Clock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the Reference Clock.
    pub fn ref_clock(&self) -> ReferenceClock {
        ReferenceClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the System Clock
    pub fn sys_clock(&self) -> SystemClock {
        SystemClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the PeripheralClock
    pub fn peripheral_clock(&self) -> PeripheralClock {
        PeripheralClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the Usb Clock
    pub fn usb_clock(&self) -> UsbClock {
        UsbClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the Adc Clock
    pub fn adc_clock(&self) -> AdcClock {
        AdcClock {
            shared_dev: self.shared_clocks,
        }
    }

    /// Getter for the Rtc Clock
    pub fn rtc_clock(&self) -> RtcClock {
        RtcClock {
            shared_dev: self.shared_clocks,
        }
    }
}

/// For clocks with an integer divider.
pub trait IntegerDivision {
    /// Set integer divider value.
    fn set_int_div(&mut self, div: usize);
    /// Get integer diveder value.
    fn get_int_div(&self) -> usize;
}

/// For clocks with a fraction divider.
pub trait FractionDivision {
    /// Set fraction divider value.
    fn set_frac_div(&mut self, div: usize);
    /// Get fraction divider value.
    fn get_frac_div(&self) -> usize;
}

/// For clocks with a divider
pub trait ClockDivision {
    /// Set integer divider value.
    fn set_div(&mut self, div: u32);
    /// Get integer diveder value.
    fn get_div(&self) -> u32;
}

/// For clocks that can have XOSC as source.
pub trait XOSCClockSource {
    /// Set XOSC as a source.
    fn set_xosc_src(&mut self);
}
/// For clocks that can have ROSC as source.
pub trait ROSCClockSource {
    /// set ROSC as a source.
    fn set_rosc_src(&mut self);
}
/// For clocks that can have ... itself (?) as a source (is that the "glitchless mux" ?)
pub trait SelfAuxClockSource {
    /// Set ...
    fn set_self_aux_src(&mut self);
}
/// For clocks that can have the Reference Clock as source.
pub trait ClockREFClockSource {
    /// Set Reference Clock as
    fn set_clkref_src(&mut self);
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

/// GPIO Output 0 Clock
pub struct GpioOutput0Clock {
    shared_dev: ShareableClocks,
}

clock_generator!(GpioOutput0Clock, clk_gpout0_ctrl);

// Clock aux sources
pll_sys_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
gpin0_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
gpin1_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
pll_usb_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
rosc_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
xosc_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clocksys_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clockusb_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clockadc_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clockrtc_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);
clockref_auxsource!(GpioOutput0Clock, clk_gpout0_ctrl);

division!(GpioOutput0Clock, clk_gpout0_div);
int_division!(GpioOutput0Clock, clk_gpout0_div, u32);
frac_division!(GpioOutput0Clock, clk_gpout0_div, u8);

/// GPIO Output 1 Clock
pub struct GpioOutput1Clock {
    shared_dev: ShareableClocks,
}

clock_generator!(GpioOutput1Clock, clk_gpout1_ctrl);

// Clock aux sources
pll_sys_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
gpin0_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
gpin1_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
pll_usb_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
rosc_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
xosc_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clocksys_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clockusb_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clockadc_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clockrtc_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);
clockref_auxsource!(GpioOutput1Clock, clk_gpout1_ctrl);

division!(GpioOutput1Clock, clk_gpout1_div);
int_division!(GpioOutput1Clock, clk_gpout1_div, u32);
frac_division!(GpioOutput1Clock, clk_gpout1_div, u8);

/// GPIO Output 2 Clock
pub struct GpioOutput2Clock {
    shared_dev: ShareableClocks,
}

clock_generator!(GpioOutput2Clock, clk_gpout2_ctrl);

// Clock aux sources
pll_sys_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
gpin0_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
gpin1_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
pll_usb_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
rosc_ph_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
xosc_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clocksys_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clockusb_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clockadc_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clockrtc_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);
clockref_auxsource!(GpioOutput2Clock, clk_gpout2_ctrl);

division!(GpioOutput2Clock, clk_gpout2_div);
int_division!(GpioOutput2Clock, clk_gpout2_div, u32);
frac_division!(GpioOutput2Clock, clk_gpout2_div, u8);

/// GPIO Output 3 Clock
pub struct GpioOutput3Clock {
    shared_dev: ShareableClocks,
}

clock_generator!(GpioOutput3Clock, clk_gpout3_ctrl);

// Clock aux sources
pll_sys_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
gpin0_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
gpin1_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
pll_usb_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
rosc_ph_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
xosc_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clocksys_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clockusb_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clockadc_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clockrtc_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);
clockref_auxsource!(GpioOutput3Clock, clk_gpout3_ctrl);

division!(GpioOutput3Clock, clk_gpout3_div);
int_division!(GpioOutput3Clock, clk_gpout3_div, u32);
frac_division!(GpioOutput3Clock, clk_gpout3_div, u8);

/// Reference Clock
pub struct ReferenceClock {
    shared_dev: ShareableClocks,
}

impl ReferenceClock {
    /// WIP - Helper function to reset source (blocking)
    pub fn reset_source_await(&mut self) {
        let shared_dev = unsafe { self.shared_dev.get() };

        shared_dev.clk_ref_ctrl.write(|w| {
            unsafe { w.src().bits(0) };
            w
        });

        self.await_select(0x0);
    }

    /// WIP - Helper function to select new source (blocking)
    pub fn await_select(&self, clock: u8) {
        let shared_dev = unsafe { self.shared_dev.get() };

        while (shared_dev.clk_ref_selected.read().bits() & (1 << clock)) == 0 {
            cortex_m::asm::nop();
        }
    }
}

rosc_source!(ReferenceClock, clk_ref_ctrl);
selfaux_source!(ReferenceClock, clk_ref_ctrl, clksrc_clk_ref_aux);
xosc_source!(ReferenceClock, clk_ref_ctrl);

// Clock aux sources
pll_usb_auxsource!(ReferenceClock, clk_ref_ctrl);
gpin0_auxsource!(ReferenceClock, clk_ref_ctrl);
gpin1_auxsource!(ReferenceClock, clk_ref_ctrl);

division!(ReferenceClock, clk_ref_div);
/// System Clock
pub struct SystemClock {
    shared_dev: ShareableClocks,
}
impl SystemClock {
    /// WIP - Helper function to reset source (blocking)
    pub fn reset_source_await(&mut self) {
        let shared_dev = unsafe { self.shared_dev.get() };

        shared_dev.clk_sys_ctrl.write(|w| {
            w.src().clear_bit();
            w
        });

        self.await_select(0x0);
    }

    /// WIP - Helper function to select new source (blocking)
    pub fn await_select(&self, clock: u8) {
        let shared_dev = unsafe { self.shared_dev.get() };

        while (shared_dev.clk_sys_selected.read().bits() & (1 << clock)) == 0 {
            cortex_m::asm::nop();
        }
    }
}

// Clock glitchless sources
clockref_source!(SystemClock, clk_sys_ctrl);
selfaux_source!(SystemClock, clk_sys_ctrl, clksrc_clk_sys_aux);

// Clock aux sources
pll_sys_auxsource!(SystemClock, clk_sys_ctrl);
pll_usb_auxsource!(SystemClock, clk_sys_ctrl);
rosc_auxsource!(SystemClock, clk_sys_ctrl);
xosc_auxsource!(SystemClock, clk_sys_ctrl);
gpin0_auxsource!(SystemClock, clk_sys_ctrl);
gpin1_auxsource!(SystemClock, clk_sys_ctrl);

division!(SystemClock, clk_sys_div);
int_division!(SystemClock, clk_sys_div, u32);
frac_division!(SystemClock, clk_sys_div, u8);

/// Peripheral Clock
pub struct PeripheralClock {
    shared_dev: ShareableClocks,
}
clock_generator!(PeripheralClock, clk_peri_ctrl);

// Clock aux sources
clocksys_auxsource!(PeripheralClock, clk_peri_ctrl);
pll_sys_auxsource!(PeripheralClock, clk_peri_ctrl);
pll_usb_auxsource!(PeripheralClock, clk_peri_ctrl);
rosc_ph_auxsource!(PeripheralClock, clk_peri_ctrl);
xosc_auxsource!(PeripheralClock, clk_peri_ctrl);
gpin0_auxsource!(PeripheralClock, clk_peri_ctrl);
gpin1_auxsource!(PeripheralClock, clk_peri_ctrl);

/// USB Clock
pub struct UsbClock {
    shared_dev: ShareableClocks,
}
clock_generator!(UsbClock, clk_usb_ctrl);

// Clock aux sources
pll_usb_auxsource!(UsbClock, clk_usb_ctrl);
pll_sys_auxsource!(UsbClock, clk_usb_ctrl);
rosc_ph_auxsource!(UsbClock, clk_usb_ctrl);
xosc_auxsource!(UsbClock, clk_usb_ctrl);
gpin0_auxsource!(UsbClock, clk_usb_ctrl);
gpin1_auxsource!(UsbClock, clk_usb_ctrl);

division!(UsbClock, clk_usb_div);

/// Adc Clock
pub struct AdcClock {
    shared_dev: ShareableClocks,
}
clock_generator!(AdcClock, clk_adc_ctrl);

// Clock aux sources
pll_usb_auxsource!(AdcClock, clk_adc_ctrl);
pll_sys_auxsource!(AdcClock, clk_adc_ctrl);
rosc_ph_auxsource!(AdcClock, clk_adc_ctrl);
xosc_auxsource!(AdcClock, clk_adc_ctrl);
gpin0_auxsource!(AdcClock, clk_adc_ctrl);
gpin1_auxsource!(AdcClock, clk_adc_ctrl);

division!(AdcClock, clk_adc_div);

/// RTC Clock
pub struct RtcClock {
    shared_dev: ShareableClocks,
}
clock_generator!(RtcClock, clk_rtc_ctrl);

// Clock aux sources
pll_usb_auxsource!(RtcClock, clk_rtc_ctrl);
pll_sys_auxsource!(RtcClock, clk_rtc_ctrl);
rosc_ph_auxsource!(RtcClock, clk_rtc_ctrl);
xosc_auxsource!(RtcClock, clk_rtc_ctrl);
gpin0_auxsource!(RtcClock, clk_rtc_ctrl);
gpin1_auxsource!(RtcClock, clk_rtc_ctrl);

division!(RtcClock, clk_rtc_div);
int_division!(RtcClock, clk_rtc_div, u32);
frac_division!(RtcClock, clk_rtc_div, u8);
