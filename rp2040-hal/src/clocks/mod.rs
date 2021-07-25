//! Clocks (CLOCKS)
//!
//!
//!
//! ## Usage simple
//! ```rust
//! let mut p = rp2040_pac::Peripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(p.WATCHDOG);
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
//! let mut clocks = init_clocks_and_plls(XOSC_CRYSTAL_FREQ, p.XOSC, p.CLOCKS, p.PLL_SYS, p.PLL_USB, &mut p.RESETS, &mut watchdog);
//! ```
//!
//! ## Usage extended
//! ```rust
//! let mut p = rp2040_pac::Peripherals::take().unwrap();
//! let mut watchdog = Watchdog::new(p.WATCHDOG);
//! let mut clocks = ClocksManager::new(p.CLOCKS, &mut watchdog);
//! // Enable the xosc
//! const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates
//! let xosc = setup_xosc_blocking(p.XOSC, XOSC_CRYSTAL_FREQ.Hz()).ok().unwrap();
//!
//! // Configure PLLs
//! //                   REF     FBDIV VCO            POSTDIV
//! // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
//! // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
//! let pll_sys = setup_pll_blocking(p.PLL_SYS, xosc.into(), PLL_SYS_125MHZ, &mut clocks, &mut p.RESETS).ok().unwrap();
//! let pll_usb = setup_pll_blocking(p.PLL_USB, xosc.into(), PLL_USB_48MHZ, &mut clocks, &mut p.RESETS).ok().unwrap();
//!
//! // Configure clocks
//! // CLK_REF = XOSC (12MHz) / 1 = 12MHz
//! self.reference_clock.configure_clock(xosc, xosc.get_freq());
//!
//! // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
//! self.system_clock.configure_clock(pll_sys, pll_sys.get_freq());
//!
//! // CLK USB = PLL USB (48MHz) / 1 = 48MHz
//! self.usb_clock.configure_clock(pll_usb, pll_usb.get_freq());
//!
//! // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
//! self.adc_clock.configure_clock(pll_usb, pll_usb.get_freq());
//!
//! // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
//! self.rtc_clock.configure_clock(pll_usb, 46875u32.Hz());
//!
//! // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
//! // Normally choose clk_sys or clk_usb
//! self.peripheral_clock.configure_clock(&self.system_clock, self.system_clock.freq());
//!
//! ```
//!
//! See [Chapter 2 Section 15](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use core::{convert::TryInto, marker::PhantomData};

use crate::{
    pll::{
        common_configs::{PLL_SYS_125MHZ, PLL_USB_48MHZ},
        setup_pll_blocking, Locked, PhaseLockedLoop,
    },
    typelevel::Sealed,
    watchdog::Watchdog,
    xosc::{setup_xosc_blocking, CrystalOscillator, Stable},
};
use embedded_time::rate::*;
use pac::{CLOCKS, PLL_SYS, PLL_USB, RESETS, XOSC};

#[macro_use]
mod macros;
mod clock_sources;

use clock_sources::PllSys;

use self::clock_sources::{GPin0, GPin1, PllUsb, Rosc, Xosc};

#[derive(Copy, Clone)]
/// Provides refs to the CLOCKS block.
struct ShareableClocks {
    _internal: (),
}

impl ShareableClocks {
    fn new(_clocks: &mut CLOCKS) -> Self {
        ShareableClocks { _internal: () }
    }

    unsafe fn get(&self) -> &pac::clocks::RegisterBlock {
        &*CLOCKS::ptr()
    }
}

/// For clocks
pub trait Clock: Sealed + Sized {
    /// Enum with valid source clocks register values for `Clock`
    type Variant;

    /// Get operating frequency
    fn freq(&self) -> Hertz;

    /// Configure this clock based on a clock source and desired frequency
    fn configure_clock<S: ValidSrc<Self>>(&mut self, src: &S, freq: Hertz) -> bool;
}

/// For clocks with a divider
trait ClockDivision {
    /// Set integer divider value.
    fn set_div(&mut self, div: u32);
    /// Get integer diveder value.
    fn get_div(&self) -> u32;
}

/// Clock with glitchless source
trait GlitchlessClock {
    /// Self type to hand to ChangingClockToken
    type Clock: Clock;

    /// Await switching clock sources without glitches. Needs a token that is returned when setting
    fn await_select(&self, clock_token: &ChangingClockToken<Self::Clock>) -> nb::Result<(), ()>;
}

/// Token which can be used to await the glitchless switch
pub struct ChangingClockToken<G: Clock> {
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

/// Trait to contrain which ClockSource is valid for which Clock
pub trait ValidSrc<C: Clock>: Sealed + ClockSource {
    /// Is this a ClockSource for src or aux?
    fn is_aux(&self) -> bool;
    /// Get register value for this ClockSource
    fn variant(&self) -> C::Variant;
}

clocks! {
    /// GPIO Output 0 Clock
    struct GpioOutput0Clock {
        init_freq: 0,
        reg: clk_gpout0,
        auxsrc: {PllSys:CLKSRC_PLL_SYS, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC, Xosc: XOSC_CLKSRC, SystemClock: CLK_SYS, UsbClock: CLK_USB, AdcClock: CLK_ADC, RtcClock: CLK_RTC, ReferenceClock:CLK_REF}
    }
    /// GPIO Output 1 Clock
    struct GpioOutput1Clock {
        init_freq: 0,
        reg: clk_gpout1,
        auxsrc: {PllSys:CLKSRC_PLL_SYS, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC, Xosc: XOSC_CLKSRC, SystemClock: CLK_SYS, UsbClock: CLK_USB, AdcClock: CLK_ADC, RtcClock: CLK_RTC, ReferenceClock:CLK_REF}
    }
    /// GPIO Output 2 Clock
    struct GpioOutput2Clock {
        init_freq: 0,
        reg: clk_gpout2,
        auxsrc: {PllSys:CLKSRC_PLL_SYS, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC, SystemClock: CLK_SYS, UsbClock: CLK_USB, AdcClock: CLK_ADC, RtcClock: CLK_RTC, ReferenceClock:CLK_REF}
    }
    /// GPIO Output 3 Clock
    struct GpioOutput3Clock {
        init_freq: 0,
        reg: clk_gpout3,
        auxsrc: {PllSys:CLKSRC_PLL_SYS, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC, SystemClock: CLK_SYS, UsbClock: CLK_USB, AdcClock: CLK_ADC, RtcClock: CLK_RTC, ReferenceClock:CLK_REF}
    }
    /// Reference Clock
    struct ReferenceClock {
        init_freq: 12_000_000,  // Starts from ROSC which actually varies with input voltage etc, but 12 MHz seems to be a good value
        reg: clk_ref,
        src: {Rosc: ROSC_CLKSRC_PH, Xosc:XOSC_CLKSRC},
        auxsrc: {PllUsb:CLKSRC_PLL_USB, GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
    /// System Clock
    struct SystemClock {
        init_freq: 12_000_000,  // ref_clk is 12 MHz
        reg: clk_sys,
        src: {ReferenceClock: CLK_REF},
        auxsrc: {PllSys: CLKSRC_PLL_SYS, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
    /// Peripheral Clock
    struct PeripheralClock {
        init_freq: 12_000_000,  // sys_clk is 12 MHz
        reg: clk_peri,
        auxsrc: {SystemClock: CLK_SYS, PllSys: CLKSRC_PLL_SYS, PllUsb:CLKSRC_PLL_USB, Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1 },
        div: false
    }
    /// USB Clock
    struct UsbClock {
        init_freq: 0,
        reg: clk_usb,
        auxsrc: {PllUsb:CLKSRC_PLL_USB,PllSys: CLKSRC_PLL_SYS,  Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
    /// Adc Clock
    struct AdcClock {
        init_freq: 0,
        reg: clk_adc,
        auxsrc: {PllUsb:CLKSRC_PLL_USB,PllSys: CLKSRC_PLL_SYS,  Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
    /// RTC Clock
    struct RtcClock {
        init_freq: 0,
        reg: clk_rtc,
        auxsrc: {PllUsb:CLKSRC_PLL_USB,PllSys: CLKSRC_PLL_SYS,  Rosc: ROSC_CLKSRC_PH, Xosc: XOSC_CLKSRC,GPin0:CLKSRC_GPIN0, GPin1:CLKSRC_GPIN1}
    }
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

impl ClocksManager {
    /// Initialize the clocks to a sane default
    pub fn init_default(
        &mut self,
        xosc: &CrystalOscillator<Stable>,
        pll_sys: &PhaseLockedLoop<Locked, PLL_SYS>,
        pll_usb: &PhaseLockedLoop<Locked, PLL_USB>,
    ) {
        // Configure clocks
        // CLK_REF = XOSC (12MHz) / 1 = 12MHz
        self.reference_clock.configure_clock(xosc, xosc.get_freq());

        // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
        self.system_clock
            .configure_clock(pll_sys, pll_sys.get_freq());

        // CLK USB = PLL USB (48MHz) / 1 = 48MHz
        self.usb_clock.configure_clock(pll_usb, pll_usb.get_freq());

        // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
        self.adc_clock.configure_clock(pll_usb, pll_usb.get_freq());

        // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
        self.rtc_clock.configure_clock(pll_usb, 46875u32.Hz());

        // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
        // Normally choose clk_sys or clk_usb
        self.peripheral_clock
            .configure_clock(&self.system_clock, self.system_clock.freq());
    }

    /// Releases the CLOCKS block
    pub fn free(self) -> CLOCKS {
        self.clocks
    }
}

/// Initialize the clocks and plls according to the reference implementation
pub fn init_clocks_and_plls(
    xosc_crystal_freq: u32,
    xosc_dev: XOSC,
    clocks_dev: CLOCKS,
    pll_sys_dev: PLL_SYS,
    pll_usb_dev: PLL_USB,
    resets: &mut RESETS,
    watchdog: &mut Watchdog,
) -> ClocksManager {
    let xosc = setup_xosc_blocking(xosc_dev, xosc_crystal_freq.Hz())
        .ok()
        .unwrap();

    // Start tick in watchdog
    watchdog.enable_tick_generation(xosc_crystal_freq as u8);

    let mut clocks = ClocksManager::new(clocks_dev);

    let pll_sys = setup_pll_blocking(
        pll_sys_dev,
        xosc.operating_frequency().into(),
        PLL_SYS_125MHZ,
        &mut clocks,
        resets,
    )
    .ok()
    .unwrap();
    let pll_usb = setup_pll_blocking(
        pll_usb_dev,
        xosc.operating_frequency().into(),
        PLL_USB_48MHZ,
        &mut clocks,
        resets,
    )
    .ok()
    .unwrap();

    clocks.init_default(&xosc, &pll_sys, &pll_usb);
    clocks
}
