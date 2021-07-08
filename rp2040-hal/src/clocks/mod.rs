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
//! // Configure PLLs
//! //                   REF     FBDIV VCO            POSTDIV
//! // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
//! // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
//! let pll_sys = setup_pll_blocking(p.PLL_SYS, 12.MHz().into(), PLL_SYS_125MHZ, &mut clocks, &mut p.RESETS).ok().unwrap();
//! let pll_usb = setup_pll_blocking(p.PLL_USB, 12.MHz().into(), PLL_USB_48MHZ, &mut clocks, &mut p.RESETS).ok().unwrap();
//!
//! // Configure clocks
//! // CLK_REF = XOSC (12MHz) / 1 = 12MHz
//! let mut ref_clock = clocks.reference_clock();
//! ref_clock.configure_clock(&xosc, xosc.get_freq());
//!
//! // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
//! let mut sys_clock = clocks.system_clock();
//! sys_clock.configure_clock(&pll_sys, pll_sys.get_freq());
//!
//! // CLK USB = PLL USB (48MHz) / 1 = 48MHz
//! let mut usb_clock = clocks.usb_clock();
//! usb_clock.configure_clock(&pll_usb, pll_usb.get_freq());
//!
//! // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
//! let mut adc_clock = clocks.adc_clock();
//! adc_clock.configure_clock(&pll_usb, pll_usb.get_freq());
//!
//! // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
//! let mut rtc_clock = clocks.rtc_clock();
//! rtc_clock.configure_clock(&pll_usb, 46875u32.Hz());
//!
//! // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
//! // Normally choose clk_sys or clk_usb
//! let mut peripheral_clock = clocks.peripheral_clock();
//! peripheral_clock.configure_clock(&sys_clock, sys_clock.freq());
//!
//! ```
//!
//! See [Chapter 2 Section 15](https://datasheets.raspberrypi.org/rp2040/rp2040_datasheet.pdf) for more details

use crate::{
    clocks::available_clocks::ClockSource,
    pll::{Locked, PhaseLockedLoop},
    watchdog::Watchdog,
    xosc::{CrystalOscillator, Stable},
};
use embedded_time::rate::*;
use pac::{CLOCKS, PLL_SYS, PLL_USB};

#[macro_use]
mod macros;
pub mod available_clocks;
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

const XOSC_MHZ: u32 = 12_000_000_u32;

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

    /// Initialize the clocks to a sane default
    pub fn init_default(
        &self,
        xosc: &CrystalOscillator<Stable>,
        pll_sys: &PhaseLockedLoop<Locked, PLL_SYS>,
        pll_usb: &PhaseLockedLoop<Locked, PLL_USB>,
    ) {
        // Configure clocks
        // CLK_REF = XOSC (12MHz) / 1 = 12MHz
        let mut ref_clock = self.reference_clock();
        ref_clock.configure_clock(xosc, xosc.get_freq());

        // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
        let mut sys_clock = self.system_clock();
        sys_clock.configure_clock(pll_sys, pll_sys.get_freq());

        // CLK USB = PLL USB (48MHz) / 1 = 48MHz
        let mut usb_clock = self.usb_clock();
        usb_clock.configure_clock(pll_usb, pll_usb.get_freq());

        // CLK ADC = PLL USB (48MHZ) / 1 = 48MHz
        let mut adc_clock = self.adc_clock();
        adc_clock.configure_clock(pll_usb, pll_usb.get_freq());

        // CLK RTC = PLL USB (48MHz) / 1024 = 46875Hz
        let mut rtc_clock = self.rtc_clock();
        rtc_clock.configure_clock(pll_usb, 46875u32.Hz());

        // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
        // Normally choose clk_sys or clk_usb
        let mut peripheral_clock = self.peripheral_clock();
        peripheral_clock.configure_clock(&sys_clock, sys_clock.freq());
    }

    /// Releases the CLOCKS block
    pub fn free(self) -> CLOCKS {
        self.clocks
    }
}
