//! Clock tree configuration
//!
//! This was my first idea on how to make a dynamic clock tree, it turned out to be quite
//! complicated. Maybe at some point we could take another look at this.
//!
//! # Example
//! The idea behind this design is that you can use the blocks of the clock as similar to normal
//! Rust as possible. This basically means you can use a clock by value, by reference, with shared
//! ownership or you can leak them (and thus freeze their state forever).
//!
//! ## Borrowed clocks
//! ```rust no_run ignore
//! use embedded_time::rate::Extensions as _;
//!
//! use rp2040_pac::Peripherals;
//! use rp2040_hal::clocks::{xosc::XOsc, unstable::SplitClocks, pll::Pll};
//!
//! let p = Peripherals::take().unwrap();
//!
//! // Setup the clock sources
//! let xosc = XOsc::new_stable_blocking(p.XOSC, 12.MHz().into()).unwrap();
//! let pll_sys = Pll::new_stable_blocking(p.PLL_SYS, &xosc, 125.MHz().into()).unwrap();
//! let pll_usb = Pll::new_stable_blocking(p.PLL_USB, &xosc, 48.MHz().into()).unwrap();
//!
//! // Derive the clocks for the different domains
//! let clocks = SplitClocks::split(p.CLOCKS);
//!
//! let clk_peri = clocks.peri;
//! // clk_peri.switch_to(&pll_sys);
//!
//! // ToDo: Use UART
//! ```
//!
//!! ## Leaked clocks
//! ```rust no_run ignore
//! use embedded_time::rate::Extensions as _;
//!
//! use rp2040_pac::Peripherals;
//! use rp2040_hal::clocks::{xosc::XOsc, SplitClocks, pll::Pll};
//!
//! let p = Peripherals::take().unwrap();
//!
//! // Setup the clock sources
//! let xosc = XOsc::new_stable_blocking(p.XOSC, 12.MHz().into()).unwrap().leak();
//! let pll_sys = Pll::new_stable_blocking(p.PLL_SYS, xosc, 125.MHz().into()).unwrap().leak();
//! let pll_usb = Pll::new_stable_blocking(p.PLL_USB, xosc, 48.MHz().into()).unwrap().leak();
//!
//! // XOsc can now go out of scope, since the PLL have leaked copies
//! drop(xosc);
//!
//! let _ = pll_usb;
//!
//! // Derive the clocks for the different domains
//! let clocks = SplitClocks::split(p.CLOCKS);
//!
//! let clk_peri = clocks.peri;
//! // clk_peri.switch_to(pll_sys);
//!
//! // ToDo: Use UART
//! ```
//!
//!! ## Shared clocks
//! ```rust no_run ignore
//! use embedded_time::rate::Extensions as _;
//!
//! use rp2040_pac::Peripherals;
//! use rp2040_hal::clocks::{XOsc, SplitClocks};
//! use rp2040_hal::clocks::pll::Pll;
//!
//! let p = Peripherals::take().unwrap();
//!
//! // Setup the clock sources
//! let mut xosc = XOsc::new_stable_blocking(p.XOSC, 12.MHz().into()).unwrap().share();
//! let pll_sys = Pll::new_stable_blocking(p.PLL_SYS, xosc.take_share(), 125.MHz().into()).unwrap().share();
//! let pll_usb = Pll::new_stable_blocking(p.PLL_USB, xosc.take_share(), 48.MHz().into()).unwrap().share();
//!
//! // ToDo: Use shared pll with device
//!
//! // Go to power down -> Shutdown PLLs and XOsc
//! let (pll_sys_dev, src) = pll_sys.un_share().disable().free();
//! xosc.return_share(src);
//! let (pll_usb_dev, src) = pll_usb.un_share().disable().free();
//! xosc.return_share(src);
//!
//! assert!(!xosc.has_outstanding_shares());
//! let disabled_xosc = xosc.un_share().disable();
//!
//! // Derive the clocks for the different domains
//! let clocks = SplitClocks::split(p.CLOCKS);
//!
//! let clk_peri = clocks.peri;
//! // clk_peri.switch_to(pll_sys);
//!
//! // ToDo: Use UART
//! ```

pub mod generators;

pub use generators::SplitClocks;

use embedded_time::rate;

/// Type used to represent rates internally to avoid spamming generic type parameters.
pub type Rate = rate::Generic<u32>;

/// Marker Trait for Clock sourcing devices
pub trait ClkDevice {}

/// Trait for wrappers around clock sources
pub trait ClkSource {
    /// The device this source is referring to. (e.g. `pac::XOSC`)
    type DeviceName;

    /// Returns the clock rate of the wrapped device.
    fn rate(&self) -> rate::Generic<u32>;
}
